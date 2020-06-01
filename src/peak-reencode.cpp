/*
 * Copyright (C) 2020  Christian Berger
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
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("in")) || (0 == commandlineArguments.count("out")) ) {
        std::cerr << argv[0] << " reencodes an existing recording file to transcode non-SI units to SI-units for PEAK GPS." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --in=<existing recording> --out=<output> [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --in=myRec.rec --out=myNewRec.rec" << std::endl;
        retCode = 1;
    } else {
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        std::fstream fin(commandlineArguments["in"], std::ios::in|std::ios::binary);
        if (!fin.good()) {
            std::cerr << "Failed to open in file." << std::endl;
            return retCode;
        }

        std::fstream fout(commandlineArguments["out"], std::ios::out|std::ios::binary);
        if (!fout.good()) {
            std::cerr << "Failed to open out file." << std::endl;
            return retCode;
        }

        const float mG_to_mps2{9.80665f/1000.f};
        const float mT_to_T{1e-6f};
        while (fin.good()) {
            auto retVal{cluon::extractEnvelope(fin)};
            if (retVal.first) {
                cluon::data::Envelope e = retVal.second;

                if (e.dataType() == opendlv::device::gps::peak::Acceleration::ID()) {
                    opendlv::device::gps::peak::Acceleration _old = cluon::extractMessage<opendlv::device::gps::peak::Acceleration>(std::move(e));
                    opendlv::device::gps::peak::Acceleration _new{_old};
                    _new.accelerationX(_old.accelerationX()*mG_to_mps2)
                        .accelerationY(_old.accelerationY()*mG_to_mps2)
                        .accelerationZ(_old.accelerationZ()*mG_to_mps2);

                    if (VERBOSE) {
                        std::stringstream sstr;
                        _new.accept([](uint32_t, const std::string &, const std::string &) {},
                                    [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                    []() {});
                        std::cout << sstr.str() << std::endl;

                    }

                    cluon::ToProtoVisitor proto;
                    _new.accept(proto);
                    e.serializedData(proto.encodedData());
                }
                else if (e.dataType() == opendlv::proxy::AccelerationReading::ID()) {
                    opendlv::proxy::AccelerationReading _old = cluon::extractMessage<opendlv::proxy::AccelerationReading>(std::move(e));
                    opendlv::proxy::AccelerationReading _new{_old};
                    _new.accelerationX(_old.accelerationX()*mG_to_mps2)
                        .accelerationY(_old.accelerationY()*mG_to_mps2)
                        .accelerationZ(_old.accelerationZ()*mG_to_mps2);

                    if (VERBOSE) {
                        std::stringstream sstr;
                        _new.accept([](uint32_t, const std::string &, const std::string &) {},
                                    [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                    []() {});
                        std::cout << sstr.str() << std::endl;

                    }

                    cluon::ToProtoVisitor proto;
                    _new.accept(proto);
                    e.serializedData(proto.encodedData());
                }
                else if (e.dataType() == opendlv::proxy::MagneticFieldReading::ID()) {
                    opendlv::proxy::MagneticFieldReading _old = cluon::extractMessage<opendlv::proxy::MagneticFieldReading>(std::move(e));
                    opendlv::proxy::MagneticFieldReading _new{_old};
                    _new.magneticFieldX(_old.magneticFieldX()*mT_to_T)
                        .magneticFieldY(_old.magneticFieldY()*mT_to_T)
                        .magneticFieldZ(_old.magneticFieldZ()*mT_to_T);

                    if (VERBOSE) {
                        std::stringstream sstr;
                        _new.accept([](uint32_t, const std::string &, const std::string &) {},
                                    [&sstr](uint32_t, std::string &&, std::string &&n, auto v) { sstr << n << " = " << v << '\n'; },
                                    []() {});
                        std::cout << sstr.str() << std::endl;

                    }

                    cluon::ToProtoVisitor proto;
                    _new.accept(proto);
                    e.serializedData(proto.encodedData());
                }
                std::string serializedData{cluon::serializeEnvelope(std::move(e))};
                fout.write(serializedData.data(), serializedData.size());
                fout.flush();
            }
        }
    }
    return retCode;
}
