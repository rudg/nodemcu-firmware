# rtctime Module

The rtctime module provides advanced timekeeping support for NodeMCU, including keeping time across deep sleep cycles (provided rtctime.dsleep() is used instead of node.dsleep()). This can be used to significantly extend battery life on battery powered sensor nodes, as it is no longer necessary to fire up the RF module each wake-up in order to obtain an accurate timestamp.

This module is intended for use together with NTP (Network Time Protocol) for keeping highly accurate real time at all times. Timestamps are available with microsecond precision, based on the Unix Epoch (1970/01/01 00:00:00).

Time keeping on the ESP8266 is technically quite challenging. Despite being named RTC, the RTC is not really a Real Time Clock in the normal sense of the word. While it does keep a counter ticking while the module is sleeping, the accuracy with which it does so is *highly* dependent on the temperature of the chip. Said temperature changes significantly between when the chip is running and when it is sleeping, meaning that any calibration performed while the chip is active becomes useless mere moments after the chip has gone to sleep. As such, calibration values need to be deduced across sleep cycles in order to enable accurate time keeping. This is one of the things this module does.

Further complicating the matter of time keeping is that the ESP8266 operates on three different clock frequencies - 52MHz right at boot, 80MHz during regular operation, and 160MHz if boosted. This module goes to considerable length to take all of this into account to properly keep the time.

To enable this module, it needs to be given a reference time at least once (via `rtctime.set()`). For best accuracy it is recommended to provide a reference time twice, with the second time being after a deep sleep.

Note that while the rtctime module can keep time across deep sleeps, it *will* lose the time if the module is unexpectedly reset.

!!! note Important:

This module uses RTC memory slots 0-9, inclusive. As soon as `rtctime.set()` (or `sntp.sync()`) has been called these RTC memory slots will be used.

####See also
  - rtcmem module
  - sntp module

## rtctime.set()

Sets the rtctime to a given timestamp in the Unix epoch (i.e. seconds from midnight 1970/01/01). If the module is not already keeping time, it starts now. If the module was already keeping time, it uses this time to help adjust its internal calibration values. Care is taken that timestamps returned from `rtctime.get()` *never go backwards*. If necessary, time is slewed and gradually allowed to catch up.

It is highly recommended that the timestamp is obtained via NTP, GPS, or other highly accurate time source.

Values very close to the epoch are not supported. This is a side effect of keeping the memory requirements as low as possible. Considering that it's no longer 1970, this is not considered a problem.

####Syntax
`rtctime.set(seconds, microseconds)`

####Parameters
  - `seconds`: the seconds part, counted from the Unix epoch.
  - `microseconds`: the microseconds part

####Returns
`nil`

####Example
```lua
rtctime.set(1436430589, 0) -- Set time to 2015 July 9, 18:29:49
```
####See also
  - `sntp.sync()`

___
## rtctime.get()

Returns the current time. If current time is not available, zero is returned.

####Syntax
`rtctime.get()`

####Parameters
`nil`

####Returns
A two-value timestamp containing:
  - `sec`: seconds since the Unix epoch
  - `usec`: the microseconds part

####Example
```lua
sec, usec = rtctime.get()
```
####See also
  - `rtctime.set()`

___
## rtctime.dsleep()

Puts the ESP8266 into deep sleep mode, like `node.dsleep()`. It differs from `node.dsleep()` in the following ways:
  - Time is kept across the deep sleep. I.e. `rtctime.get()` will keep working (provided time was available before the sleep)
  - This call never returns. The module is put to sleep immediately. This is both to support accurate time keeping and to reduce power consumption.
  - The time slept will generally be considerably more accurate than with `node.dsleep()`.
  - A sleep time of zero does not mean indefinite sleep, it is interpreted as a zero length sleep instead.

####Syntax
`rtctime.dsleep(microseconds [, option])`

####Parameters
  - `microseconds`: The number of microseconds to sleep for. Maxmium value is 4294967295us, or ~71 minutes.
  - `option`: The sleep option. See `node.dsleep()` for specifics.

####Returns
This function does not return.

####Example
```lua
rtctime.dsleep(60*1000000) -- sleep for a minute
```
```lua
rtctime.dsleep(5000000, 4) -- sleep for 5 seconds, do not start RF on wakeup
```
___
## rtctime.dsleep_aligned()

For applications where it is necessary to take samples with high regularity, this function is useful. It provides an easy way to implement a "wake up on the next 5-minute boundary" scheme, without having to explicitly take into account how long the module has been active for etc before going back to sleep.

####Syntax
`rtctime.dsleep(aligned_us, minsleep_us [, option])`

####Parameters
  - `aligned_us`: specifies the boundary interval in microseconds.
  - `minsleep_us`: minimum time that will be slept, if necessary skipping an interval. This is intended for sensors where a sample reading is started before putting the ESP8266 to sleep, and then fetched upon wake-up. Here `minsleep_us` should be the minimum time required for the sensor to take the sample.
  - `option`: as with `dsleep()`, the `option` sets the sleep option, if specified.

####Example
```lua
rtctime.dsleep_aligned(5*1000000, 3*1000000) -- sleep at least 3 seconds, then wake up on the next 5-second boundary
```
___