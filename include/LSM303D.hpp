/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <ModuleConfiguration.hpp>

#include <core/hw/SPI.hpp>
#include <core/hw/EXT.hpp>
#include <core/mw/CoreSensor.hpp>
#include <core/os/Thread.hpp>

namespace sensors {
class LSM303D
{
public:
   LSM303D(
      core::hw::SPIDevice&  spi,
      core::hw::EXTChannel& extA,
      core::hw::EXTChannel& extM
   );

   virtual
   ~LSM303D();

public:
   bool
   probe();

   uint8_t
   readRegister(
      uint8_t reg
   );

   void
   writeRegister(
      uint8_t reg,
      uint8_t value
   );


public:
   core::hw::SPIDevice&  _spi;
   core::hw::EXTChannel& _extA;
   core::hw::EXTChannel& _extM;
};

class LSM303D_Acc:
   public core::mw::CoreSensor<ModuleConfiguration::LSM303D_ACC_DATATYPE>
{
public:
   LSM303D_Acc(
      LSM303D& device
   );

   virtual
   ~LSM303D_Acc();

private:
public:
   bool
   init();

   bool
   configure();

   bool
   start();

   bool
   stop();

   bool
   waitUntilReady();

   bool
   update();

   void
   get(
      DataType& data
   );


protected:
   core::os::Thread* _runner;
   core::os::Time    _timestamp;

private:
   LSM303D& _device;
   ModuleConfiguration::LSM303D_ACC_DATATYPE _data;
};

class LSM303D_Mag:
   public core::mw::CoreSensor<ModuleConfiguration::LSM303D_MAG_DATATYPE>
{
public:
   LSM303D_Mag(
      LSM303D& device
   );

   virtual
   ~LSM303D_Mag();

private:
public:
   bool
   init();

   bool
   configure();

   bool
   start();

   bool
   stop();

   bool
   waitUntilReady();

   bool
   update();

   void
   get(
      DataType& data
   );


protected:
   core::os::Thread* _runner;
   core::os::Time    _timestamp;

private:
   LSM303D& _device;
   ModuleConfiguration::LSM303D_MAG_DATATYPE _data;
};
}
