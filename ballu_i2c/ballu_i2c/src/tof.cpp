#include <iostream>
#include <unistd.h>
#include <cstdlib> // exit
#include <cmath> //NAN
#include <ctime>

#include "tof.h"
#include <pigpiod_if2.h>

// Defines ///////////////////////////////////////////////////////////////////
// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = static_cast<uint64_t> (time(nullptr)*1000))

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)(static_cast<uint64_t>(time(nullptr)*1000) - timeout_start_ms) > io_timeout))

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


VL53LX::VL53LX(int i2c_dev, int bus_num, int16_t dev_id, bool wrapToPi, bool inRadian)
    : I2CDevice(i2c_dev, bus_num, dev_id) {_init();}
void VL53LX::_init() { 
  if(!init()) {
    cout << "Failed to detect and initialize TOF sensor!" << endl;
  }
}
VL53LX::~VL53LX(void) {}

void VL53LX::setId(uint8_t new_id)
{
  _write(I2C_SLAVE_DEVICE_ADDRESS, 1, new_id & 0x7F);
  id = new_id;
}

void VL53LX::reset() {} // Need to XSHUT pin. Feature skipped.

int VL53LX::ping(bool verbose) {
    cout << "ping will be later" << endl;
    return 1;
}

void VL53LX::update() {
  static uint16_t _range;
  _range = readRangeSingleMillimeters();
  if(_range != 65535 && _range < 5000) range = 1e-3 * _range;
  
  // if(tof->timeoutOccurred()) {
  //   cout << "TimeOut!" << endl;
  // }
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool VL53LX::init(bool io_2v8)
{
  // check model ID register (value specified in datasheet)
  if (_read(IDENTIFICATION_MODEL_ID, 1) != 0xEE) { return false; }

  // VL53LX_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    _write(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 1, _read(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 1) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  _write(0x88, 1, 0x00);

  _write(0x80, 1, 0x01);
  _write(0xFF, 1, 0x01);
  _write(0x00, 1, 0x00);
  stop_variable = _read(0x91, 1);
  _write(0x00, 1, 0x01);
  _write(0xFF, 1, 0x00);
  _write(0x80, 1, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  _write(MSRC_CONFIG_CONTROL, 1, _read(MSRC_CONFIG_CONTROL, 1) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(0.25);

  _write(SYSTEM_SEQUENCE_CONFIG, 1, 0xFF);

  // VL53LX_DataInit() end

  // VL53LX_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53LX_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  _read(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

  // -- VL53LX_set_reference_spads() begin (assume NVM values are valid)

  _write(0xFF, 1, 0x01);
  _write(DYNAMIC_SPAD_REF_EN_START_OFFSET, 1, 0x00);
  _write(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 1, 0x2C);
  _write(0xFF, 1, 0x00);
  _write(GLOBAL_CONFIG_REF_EN_START_SELECT, 1, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  _write(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, -1, ref_spad_map);
  
  // -- VL53LX_set_reference_spads() end

  // -- VL53LX_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  _write(0xFF, 1, 0x01);
  _write(0x00, 1, 0x00);

  _write(0xFF, 1, 0x00);
  _write(0x09, 1, 0x00);
  _write(0x10, 1, 0x00);
  _write(0x11, 1, 0x00);

  _write(0x24, 1, 0x01);
  _write(0x25, 1, 0xFF);
  _write(0x75, 1, 0x00);

  _write(0xFF, 1, 0x01);
  _write(0x4E, 1, 0x2C);
  _write(0x48, 1, 0x00);
  _write(0x30, 1, 0x20);

  _write(0xFF, 1, 0x00);
  _write(0x30, 1, 0x09);
  _write(0x54, 1, 0x00);
  _write(0x31, 1, 0x04);
  _write(0x32, 1, 0x03);
  _write(0x40, 1, 0x83);
  _write(0x46, 1, 0x25);
  _write(0x60, 1, 0x00);
  _write(0x27, 1, 0x00);
  _write(0x50, 1, 0x06);
  _write(0x51, 1, 0x00);
  _write(0x52, 1, 0x96);
  _write(0x56, 1, 0x08);
  _write(0x57, 1, 0x30);
  _write(0x61, 1, 0x00);
  _write(0x62, 1, 0x00);
  _write(0x64, 1, 0x00);
  _write(0x65, 1, 0x00);
  _write(0x66, 1, 0xA0);

  _write(0xFF, 1, 0x01);
  _write(0x22, 1, 0x32);
  _write(0x47, 1, 0x14);
  _write(0x49, 1, 0xFF);
  _write(0x4A, 1, 0x00);

  _write(0xFF, 1, 0x00);
  _write(0x7A, 1, 0x0A);
  _write(0x7B, 1, 0x00);
  _write(0x78, 1, 0x21);

  _write(0xFF, 1, 0x01);
  _write(0x23, 1, 0x34);
  _write(0x42, 1, 0x00);
  _write(0x44, 1, 0xFF);
  _write(0x45, 1, 0x26);
  _write(0x46, 1, 0x05);
  _write(0x40, 1, 0x40);
  _write(0x0E, 1, 0x06);
  _write(0x20, 1, 0x1A);
  _write(0x43, 1, 0x40);

  _write(0xFF, 1, 0x00);
  _write(0x34, 1, 0x03);
  _write(0x35, 1, 0x44);

  _write(0xFF, 1, 0x01);
  _write(0x31, 1, 0x04);
  _write(0x4B, 1, 0x09);
  _write(0x4C, 1, 0x05);
  _write(0x4D, 1, 0x04);

  _write(0xFF, 1, 0x00);
  _write(0x44, 1, 0x00);
  _write(0x45, 1, 0x20);
  _write(0x47, 1, 0x08);
  _write(0x48, 1, 0x28);
  _write(0x67, 1, 0x00);
  _write(0x70, 1, 0x04);
  _write(0x71, 1, 0x01);
  _write(0x72, 1, 0xFE);
  _write(0x76, 1, 0x00);
  _write(0x77, 1, 0x00);

  _write(0xFF, 1, 0x01);
  _write(0x0D, 1, 0x01);

  _write(0xFF, 1, 0x00);
  _write(0x80, 1, 0x01);
  _write(0x01, 1, 0xF8);

  _write(0xFF, 1, 0x01);
  _write(0x8E, 1, 0x01);
  _write(0x00, 1, 0x01);
  _write(0xFF, 1, 0x00);
  _write(0x80, 1, 0x00);

  // -- VL53LX_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53LX_SetGpioConfig() begin

  _write(SYSTEM_INTERRUPT_CONFIG_GPIO, 1, 0x04);
  _write(GPIO_HV_MUX_ACTIVE_HIGH, 1, _read(GPIO_HV_MUX_ACTIVE_HIGH, 1) & ~0x10); // active low
  _write(SYSTEM_INTERRUPT_CLEAR, 1, 0x01);

  // -- VL53LX_SetGpioConfig() end

  measurement_timing_budget_us = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53LX_SetSequenceStepEnable() begin

  _write(SYSTEM_SEQUENCE_CONFIG, 1, 0xE8);

  // -- VL53LX_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53LX_StaticInit() end

  // VL53LX_PerformRefCalibration() begin (VL53LX_perform_ref_calibration())

  // -- VL53LX_perform_vhv_calibration() begin

  _write(SYSTEM_SEQUENCE_CONFIG, 1, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  // -- VL53LX_perform_vhv_calibration() end

  // -- VL53LX_perform_phase_calibration() begin

  _write(SYSTEM_SEQUENCE_CONFIG, 1, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }

  // -- VL53LX_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  _write(SYSTEM_SEQUENCE_CONFIG, 1, 0xE8);

  // VL53LX_PerformRefCalibration() end

  return true;
}


// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool VL53LX::setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  _write(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 2, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
float VL53LX::getSignalRateLimit()
{
  return (float)_read(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 2) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53LX_set_measurement_timing_budget_micro_seconds()
bool VL53LX::setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53LX_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint32_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    _write(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 2, encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53LX_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53LX::getMeasurementTimingBudget()
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53LX_set_vcsel_pulse_period()
bool VL53LX::setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        _write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x18);
        break;

      case 14:
        _write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x30);
        break;

      case 16:
        _write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x40);
        break;

      case 18:
        _write(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    _write(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 1, 0x08);

    // apply new VCSEL period
    _write(PRE_RANGE_CONFIG_VCSEL_PERIOD, 1, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53LX_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    _write(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 2, encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53LX_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    _write(MSRC_CONFIG_TIMEOUT_MACROP, 1,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x10);
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 0x08);
        _write(GLOBAL_CONFIG_VCSEL_WIDTH, 1, 0x02);
        _write(ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 0x0C);
        _write(0xFF, 1, 0x01);
        _write(ALGO_PHASECAL_LIM, 1, 0x30);
        _write(0xFF, 1, 0x00);
        break;

      case 10:
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x28);
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 0x08);
        _write(GLOBAL_CONFIG_VCSEL_WIDTH, 1, 0x03);
        _write(ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 0x09);
        _write(0xFF, 1, 0x01);
        _write(ALGO_PHASECAL_LIM, 1, 0x20);
        _write(0xFF, 1, 0x00);
        break;

      case 12:
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x38);
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 0x08);
        _write(GLOBAL_CONFIG_VCSEL_WIDTH, 1, 0x03);
        _write(ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 0x08);
        _write(0xFF, 1, 0x01);
        _write(ALGO_PHASECAL_LIM, 1, 0x20);
        _write(0xFF, 1, 0x00);
        break;

      case 14:
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 0x48);
        _write(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 0x08);
        _write(GLOBAL_CONFIG_VCSEL_WIDTH, 1, 0x03);
        _write(ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 0x07);
        _write(0xFF, 1, 0x01);
        _write(ALGO_PHASECAL_LIM, 1, 0x20);
        _write(0xFF, 1, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    _write(FINAL_RANGE_CONFIG_VCSEL_PERIOD, 1, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53LX_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    _write(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 2, encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53LX_perform_phase_calibration() begin

  uint8_t sequence_config = _read(SYSTEM_SEQUENCE_CONFIG, 1);
  _write(SYSTEM_SEQUENCE_CONFIG, 1, 0x02);
  performSingleRefCalibration(0x0);
  _write(SYSTEM_SEQUENCE_CONFIG, 1, sequence_config);

  // VL53LX_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53LX_get_vcsel_pulse_period()
uint8_t VL53LX::getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(_read(PRE_RANGE_CONFIG_VCSEL_PERIOD, 1));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(_read(FINAL_RANGE_CONFIG_VCSEL_PERIOD, 1));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53LX_StartMeasurement()
void VL53LX::startContinuous(uint32_t period_ms)
{
  _write(0x80, 1, 0x01);
  _write(0xFF, 1, 0x01);
  _write(0x00, 1, 0x00);
  _write(0x91, 1, stop_variable);
  _write(0x00, 1, 0x01);
  _write(0xFF, 1, 0x00);
  _write(0x80, 1, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53LX_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = _read(OSC_CALIBRATE_VAL, 2);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    _write(SYSTEM_INTERMEASUREMENT_PERIOD, 4, period_ms);

    // VL53LX_SetInterMeasurementPeriodMilliSeconds() end

    _write(SYSRANGE_START, 1, 0x04); // VL53LX_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    _write(SYSRANGE_START, 1, 0x02); // VL53LX_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53LX_StopMeasurement()
void VL53LX::stopContinuous()
{
  _write(SYSRANGE_START, 1, 0x01); // VL53LX_REG_SYSRANGE_MODE_SINGLESHOT

  _write(0xFF, 1, 0x01);
  _write(0x00, 1, 0x00);
  _write(0x91, 1, 0x00);
  _write(0x00, 1, 0x01);
  _write(0xFF, 1, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53LX::readRangeContinuousMillimeters()
{
  startTimeout();
  while ((_read(RESULT_INTERRUPT_STATUS, 1) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = _read(RESULT_RANGE_STATUS + 10, 2);

  _write(SYSTEM_INTERRUPT_CLEAR, 1, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53LX_PerformSingleRangingMeasurement()
uint16_t VL53LX::readRangeSingleMillimeters()
{
  _write(0x80, 1, 0x01);
  _write(0xFF, 1, 0x01);
  _write(0x00, 1, 0x00);
  _write(0x91, 1, stop_variable);
  _write(0x00, 1, 0x01);
  _write(0xFF, 1, 0x00);
  _write(0x80, 1, 0x00);

  _write(SYSRANGE_START, 1, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (_read(SYSRANGE_START, 1) & 0x01)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  return readRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53LX::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53LX_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53LX::getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  _write(0x80, 1, 0x01);
  _write(0xFF, 1, 0x01);
  _write(0x00, 1, 0x00);

  _write(0xFF, 1, 0x06);
  _write(0x83, 1, _read(0x83, 1) | 0x04);
  _write(0xFF, 1, 0x07);
  _write(0x81, 1, 0x01);

  _write(0x80, 1, 0x01);

  _write(0x94, 1, 0x6b);
  _write(0x83, 1, 0x00);
  startTimeout();
  while (_read(0x83, 1) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
  _write(0x83, 1, 0x01);
  tmp = _read(0x92, 1);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  _write(0x81, 1, 0x00);
  _write(0xFF, 1, 0x06);
  _write(0x83, 1, _read(0x83, 1) & ~0x04);
  _write(0xFF, 1, 0x01);
  _write(0x00, 1, 0x01);

  _write(0xFF, 1, 0x00);
  _write(0x80, 1, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53LX_GetSequenceStepEnables()
void VL53LX::getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = _read(SYSTEM_SEQUENCE_CONFIG, 1);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void VL53LX::getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = _read(MSRC_CONFIG_TIMEOUT_MACROP, 1) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(_read(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 2));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(_read(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 2));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53LX_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53LX::decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53LX_encode_timeout()
uint16_t VL53LX::encodeTimeout(uint32_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53LX_calc_timeout_us()
uint32_t VL53LX::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53LX_calc_timeout_mclks()
uint32_t VL53LX::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53LX_perform_single_ref_calibration()
bool VL53LX::performSingleRefCalibration(uint8_t vhv_init_byte)
{
  _write(SYSRANGE_START, 1, 0x01 | vhv_init_byte); // VL53LX_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((_read(RESULT_INTERRUPT_STATUS, 1) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return false; }
  }

  _write(SYSTEM_INTERRUPT_CLEAR, 1, 0x01);

  _write(SYSRANGE_START, 1, 0x00);

  return true;
}
