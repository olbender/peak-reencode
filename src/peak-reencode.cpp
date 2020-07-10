/*
 * Copyright (C) 2020  Christian Berger, Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "peak-gps.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

bool processRecFile(std::string const &inPath, std::string const &outPath,
    std::string const &filename, bool const verbose)
{
  {
    std::filesystem::path out = outPath + "/" + filename;
    if (std::filesystem::exists(out)) {
      if (verbose) {
        std::cout << filename << std::endl;
        std::cout << " .. exists in destination, skipping." << std::endl;
      }
      return true;
    }
  }
  
  std::fstream fin(inPath + "/" + filename, std::ios::in|std::ios::binary);
  if (!fin.good()) {
    std::cerr << "Failed to open in file." << std::endl;
    return false;
  }

  bool isBeforeSiPatch = false;
  bool isFromBrokenPatch = false;
  bool removeSwitchStateReadings = false;
  bool isFine = true;
  {
    double lengthSum{0.0};

    double xPrev{0.0};
    double yPrev{0.0};
    double zPrev{0.0};
    
    double xChangeMax{0.0};
    double yChangeMax{0.0};
    double zChangeMax{0.0};
    
    uint64_t sampleCount{0};
    while (fin.good()) {
      auto retVal{cluon::extractEnvelope(fin)};
      if (retVal.first) {
        cluon::data::Envelope e = retVal.second;

        if (e.dataType() == opendlv::proxy::AccelerationReading::ID()) {
          opendlv::proxy::AccelerationReading msg = 
            cluon::extractMessage<opendlv::proxy::AccelerationReading>(
                std::move(e));

          removeSwitchStateReadings = true;

          double x = msg.accelerationX();
          double y = msg.accelerationY();
          double z = msg.accelerationZ();

          lengthSum += std::sqrt(x * x + y * y + z * z);

          if (sampleCount != 0) {
            double xChange = std::abs(x - xPrev);
            if (xChange > xChangeMax) {
              xChangeMax = xChange;
            }
            double yChange = std::abs(y - yPrev);
            if (yChange > yChangeMax) {
              yChangeMax = yChange;
            }
            double zChange = std::abs(z - zPrev);
            if (zChange > zChangeMax) {
              zChangeMax = zChange;
            }
          }
    
          xPrev = x;
          yPrev = y;
          zPrev = z;
          
          sampleCount++;
        }
      }
    }
    double lengthMean = lengthSum / sampleCount;

    isFromBrokenPatch = (xChangeMax > 2500.0 || yChangeMax > 2500.0 
        || zChangeMax > 2500.0);
    if (!isFromBrokenPatch) {
      isBeforeSiPatch = (lengthMean > 1000.0 && lengthMean < 1060);
    }
    isFine = (!isBeforeSiPatch && !isFromBrokenPatch 
        && !removeSwitchStateReadings);

    if (verbose) {
      std::cout << filename << std::endl;
      if (isBeforeSiPatch) {
        std::cout << " .. is not in SI units, re-scaling." << std::endl;
      }
      if (isFromBrokenPatch) {
        std::cout << " .. the broken patch was used, fixing." << std::endl;
      }
      if (isFine) {
        std::cout << " .. no errors detected, copy only." << std::endl;
      } 
      if (removeSwitchStateReadings) {
        std::cout << " .. will remove switch state readings." << std::endl;
      }
    }
  }

  if (isFine) {
    std::filesystem::path in = inPath + "/" + filename;
    std::filesystem::path out = outPath + "/" + filename;
    std::filesystem::copy_file(in, out);
    fin.close();
    return true;
  }

  fin.clear();
  fin.seekg(0, std::ios::beg);

  std::fstream fout(outPath + "/" + filename, std::ios::out|std::ios::binary);
  if (!fout.good()) {
    std::cerr << "Failed to open out file." << std::endl;
    return false;
  }
  if (!fin.good()) {
    std::cerr << "Failed to open in file." << std::endl;
    return false;
  }
  fin.close();

  // Conversion constants.
  float const mG_to_mps2{9.80665f/1000.f};
  float const mT_to_T{1e-6f};

  // We need the Envelopes in strictly ascending temporal order, so
  // we use cluon::Player to sort the Envelopes by sampleTimePoint.
  const bool AUTOREWIND{false};
  const bool THREADING{false};
  cluon::Player player(inPath + "/" + filename, AUTOREWIND, THREADING);

  // temp buffer to remove duplicated values
  bool foundAngularVelocityReading{false};
  double prevAngularVelocityX{0};
  double prevAngularVelocityY{0};
  double prevAngularVelocityZ{0};
  uint32_t skippedAngularVelocityReadingsCounter{0};

  bool foundMagneticFieldReading{false};
  double prevMagneticFieldX{0};
  double prevMagneticFieldY{0};
  double prevMagneticFieldZ{0};
  uint32_t skippedMagneticFieldReadingsCounter{0};

  bool foundAltitudeReading{false};
  double prevAltitude{0};
  uint32_t skippedAltitudeReadingsCounter{0};

  bool foundGroundSpeedReading{false};
  double prevGroundSpeed{0};
  uint32_t skippedGroundSpeedReadingsCounter{0};

  bool foundGeodeticHeadingReading{false};
  double prevGeodeticHeading{0};
  uint32_t skippedGeodeticHeadingReadingsCounter{0};

  while (player.hasMoreData()) {
    auto retVal{player.getNextEnvelopeToBeReplayed()};
    if (retVal.first) {
      cluon::data::Envelope e = retVal.second;
      
      if (e.dataType() == opendlv::proxy::SwitchStateReading::ID() 
          && removeSwitchStateReadings) {
        continue;
      }
      else if (e.dataType() == opendlv::device::gps::peak::Acceleration::ID()) {
        opendlv::device::gps::peak::Acceleration _old 
          = cluon::extractMessage<opendlv::device::gps::peak::Acceleration>(
              std::move(e));

        opendlv::device::gps::peak::Acceleration _new{_old};

        if (isBeforeSiPatch) {
          _new.accelerationX(_old.accelerationX()*mG_to_mps2)
            .accelerationY(_old.accelerationY()*mG_to_mps2)
            .accelerationZ(_old.accelerationZ()*mG_to_mps2);
        }

        if (isFromBrokenPatch) {
          float x = _old.accelerationX();
          float y = _old.accelerationY();
          float z = _old.accelerationZ();
          if (x > 1250.0f) {
            x -= 2512.874f;
          }
          if (y > 1250.0f) {
            y -= 2512.874f;
          }
          if (z > 1250.0f) {
            z -= 2512.874f;
          }
          _new.accelerationX(x)
            .accelerationY(y)
            .accelerationZ(z);
        }

        cluon::ToProtoVisitor proto;
        _new.accept(proto);
        e.serializedData(proto.encodedData());
      }
      else if (e.dataType() == opendlv::proxy::AccelerationReading::ID()) {
        opendlv::proxy::AccelerationReading _old = 
          cluon::extractMessage<opendlv::proxy::AccelerationReading>(
              std::move(e));
        
        opendlv::proxy::AccelerationReading _new{_old};

        if (isBeforeSiPatch) {
          _new.accelerationX(_old.accelerationX()*mG_to_mps2)
            .accelerationY(_old.accelerationY()*mG_to_mps2)
            .accelerationZ(_old.accelerationZ()*mG_to_mps2);
        }
        
        if (isFromBrokenPatch) {
          float x = _old.accelerationX();
          float y = _old.accelerationY();
          float z = _old.accelerationZ();
          if (x > 1250.0f) {
            x -= 2512.874f;
          }
          if (y > 1250.0f) {
            y -= 2512.874f;
          }
          if (z > 1250.0f) {
            z -= 2512.874f;
          }
          _new.accelerationX(x)
            .accelerationY(y)
            .accelerationZ(z);
        }

        cluon::ToProtoVisitor proto;
        _new.accept(proto);
        e.serializedData(proto.encodedData());
      }
      else if (e.dataType() == opendlv::proxy::MagneticFieldReading::ID()) {
        opendlv::proxy::MagneticFieldReading _old 
          = cluon::extractMessage<opendlv::proxy::MagneticFieldReading>(
              std::move(e));

        // Do we need to skip this due to a duplicated value?
        {
          double x = _old.magneticFieldX();
          double y = _old.magneticFieldY();
          double z = _old.magneticFieldZ();
          if (foundMagneticFieldReading) {
            if (::memcmp(&x, &prevMagneticFieldX, 8) == 0
                || ::memcmp(&y, &prevMagneticFieldY, 8) == 0
                || ::memcmp(&z, &prevMagneticFieldZ, 8) == 0) {
              skippedMagneticFieldReadingsCounter++;
              continue;
            }
          }
          foundMagneticFieldReading = true;
          prevMagneticFieldX = x;
          prevMagneticFieldY = y;
          prevMagneticFieldZ = z;
        }
        
        opendlv::proxy::MagneticFieldReading _new{_old};

        if (isBeforeSiPatch) {
          _new.magneticFieldX(_old.magneticFieldX()*mT_to_T)
            .magneticFieldY(_old.magneticFieldY()*mT_to_T)
            .magneticFieldZ(_old.magneticFieldZ()*mT_to_T);
        }
        
        if (isFromBrokenPatch) {
          float x = _old.magneticFieldX();
          float y = _old.magneticFieldY();
          float z = _old.magneticFieldZ();
          if (x > 0.01f) {
            x -= 0.0196605f;
          }
          if (y > 0.01f) {
            y -= 0.0196605f;
          }
          if (z > 0.01f) {
            z -= 0.0196605f;
          }
          _new.magneticFieldX(x)
            .magneticFieldY(y)
            .magneticFieldZ(z);
        }

        cluon::ToProtoVisitor proto;
        _new.accept(proto);
        e.serializedData(proto.encodedData());
      }
      else if (e.dataType() == opendlv::proxy::AngularVelocityReading::ID()) {
        opendlv::proxy::AngularVelocityReading _old 
          = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(
              std::move(e));

        // Do we need to skip this due to a duplicated value?
        {
          double x = _old.angularVelocityX();
          double y = _old.angularVelocityY();
          double z = _old.angularVelocityZ();
          if (foundAngularVelocityReading) {
            if (::memcmp(&x, &prevAngularVelocityX, 8) == 0
                || ::memcmp(&y, &prevAngularVelocityY, 8) == 0
                || ::memcmp(&z, &prevAngularVelocityZ, 8) == 0) {
              skippedAngularVelocityReadingsCounter++;
              continue;
            }
          }
          foundAngularVelocityReading = true;
          prevAngularVelocityX = x;
          prevAngularVelocityY = y;
          prevAngularVelocityZ = z;
        }
        
        opendlv::proxy::AngularVelocityReading _new{_old};

        cluon::ToProtoVisitor proto;
        _new.accept(proto);
        e.serializedData(proto.encodedData());
      }
      if (e.dataType() == opendlv::proxy::AltitudeReading::ID()) {
        auto msg = cluon::extractMessage<opendlv::proxy::AltitudeReading>(std::move(e));
        double x = msg.altitude();
        if (foundAltitudeReading) {
          if (prevAltitude - x >  0.98 * std::abs(prevAltitude)) {
            skippedAltitudeReadingsCounter++;
            continue;
          }
          if (::memcmp(&x, &prevAltitude, 8) == 0) {
            skippedAltitudeReadingsCounter++;
            continue;
          }
        }
        foundAltitudeReading = true;
        prevAltitude = x;

        cluon::ToProtoVisitor proto;
        msg.accept(proto);
        e.serializedData(proto.encodedData());
      }
      if (e.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
        auto msg = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(e));
        double x = msg.groundSpeed();
        if (foundGroundSpeedReading) {
          if (prevGroundSpeed - x >  0.98 * std::abs(prevGroundSpeed)) {
            skippedGroundSpeedReadingsCounter++;
            continue;
          }
          if (::memcmp(&x, &prevGroundSpeed, 8) == 0) {
            skippedGroundSpeedReadingsCounter++;
            continue;
          }
        }
        foundGroundSpeedReading = true;
        prevGroundSpeed = x;

        cluon::ToProtoVisitor proto;
        msg.accept(proto);
        e.serializedData(proto.encodedData());
      }
      if (e.dataType() == opendlv::proxy::GeodeticHeadingReading::ID()) {
        auto msg = cluon::extractMessage<opendlv::proxy::GeodeticHeadingReading>(std::move(e));
        double x = msg.northHeading();
        if (std::abs(x) < 0.001) {
          continue;
        }
        if (foundGeodeticHeadingReading) {
          if (prevGeodeticHeading - x >  0.98 * std::abs(prevGeodeticHeading)) {
            skippedGeodeticHeadingReadingsCounter++;
            continue;
          }
          if (::memcmp(&x, &prevGeodeticHeading, 8) == 0) {
            skippedGeodeticHeadingReadingsCounter++;
            continue;
          }
        }
        foundGeodeticHeadingReading = true;
        prevGeodeticHeading = x;

        cluon::ToProtoVisitor proto;
        msg.accept(proto);
        e.serializedData(proto.encodedData());
      }

      std::string serializedData{cluon::serializeEnvelope(std::move(e))};
      fout.write(serializedData.data(), serializedData.size());
      fout.flush();
    }
  }
  if (verbose) {
    std::cout << "..skipped " << skippedMagneticFieldReadingsCounter << " duplicated MagneticFieldReadings" << std::endl;
    std::cout << "..skipped " << skippedAngularVelocityReadingsCounter << " duplicated AngularVelocityReadings" << std::endl;
    std::cout << "..skipped " << skippedAltitudeReadingsCounter << " duplicated or invalid AltitudeReadings" << std::endl;
    std::cout << "..skipped " << skippedGroundSpeedReadingsCounter << " duplicated or invalid GroundSpeedReadings" << std::endl;
    std::cout << "..skipped " << skippedGeodeticHeadingReadingsCounter << " duplicated or invalid GeodeticHeadingReadings" << std::endl;
  }

  fout.close();
  return true;
}


int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("in")) 
      || (0 == commandlineArguments.count("out")) ) {
    std::cerr << argv[0] << " reencodes an existing recording file to "
      << "transcode non-SI units to SI-units for PEAK GPS." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --in=<existing folder with recordings> "
      << "--out=<output folder> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --in=in-rec --out=out-rec" 
      << std::endl;
    retCode = 1;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};

    std::filesystem::path inPath = commandlineArguments["in"] + "/";
    std::filesystem::path outPath = commandlineArguments["out"] + "/";

    if (inPath == outPath) {
      std::cerr << "ERROR: Cannot re-save files to source directory" 
        << std::endl;
      return -1;
    }

    std::string inPathAbs = std::filesystem::absolute(inPath).string();
    std::string outPathAbs = std::filesystem::absolute(outPath).string();

    for (auto const &entry : 
        std::filesystem::recursive_directory_iterator(inPath)) {

      if (entry.is_regular_file() && entry.path().extension() == ".rec") {
        std::string filename = entry.path().string();
        std::string relativeFilename = filename.substr(inPathAbs.length());

        std::filesystem::path out = outPath.string() + relativeFilename;
        std::filesystem::create_directories(out.parent_path());

        bool ok = processRecFile(inPathAbs, outPathAbs, relativeFilename, verbose);
        if (!ok) {
          return -1;
        }

      }
    }
  }
  return retCode;
}
