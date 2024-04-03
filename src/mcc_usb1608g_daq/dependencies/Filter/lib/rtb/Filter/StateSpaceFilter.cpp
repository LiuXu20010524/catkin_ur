/* -------------------------------------------------------------------------- *
 * Copyright (c) 2010-2016                                                    *
 * T. van den Bogert, E. Ceseracciu, C. Pizzolato, M. Reggiani                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at:                                   *
 * http://www.apache.org/licenses/LICENSE-2.0                                 *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <vector>
#include <iostream>
#include <limits>
#include <cmath>

namespace rtb {
    namespace Filter {

        template<typename T>
        StateSpaceFilter<T>::StateSpaceFilter() :
            lastTime_(std::numeric_limits<double>::infinity()),
            lastValue_(0.0),
            lastFirstDerivative_(0.0),
            lastSecondDerivative_(0.0)
        { }

        template<typename T>
        T StateSpaceFilter<T>::filter(T value, double time, double f0) {

            if (time < lastTime_)
            {
                lastValue_ = value;
                lastFirstDerivative_ = 0.0;
                lastSecondDerivative_ = 0.0;
            }
            else
            {
                double h = time - lastTime_;
                double a = (2 * M_PI * f0) * (2 * M_PI * f0);
                double b = sqrt(2) * 2 * M_PI * f0;
                double denom = 4 + 2 * h * b + h * h * a;
                double A = (4 + 2 * h*b - h * h * a) / denom;
                double B = 4 * h / denom;
                double C = -4 * h*a / denom;
                double D = (4 - 2 * h * b - h * h * a) / denom;
                double E = 2 * h * h * a / denom;
                double F = 4 * h*a / denom;
                double y = A*lastValue_ + B*lastFirstDerivative_ + E*(value + lastValue_) / 2;
                double yd = C*lastValue_ + D*lastFirstDerivative_ + F*(value + lastValue_) / 2;
                lastSecondDerivative_ = (yd - lastFirstDerivative_) / h;

                lastValue_ = y;
                lastFirstDerivative_ = yd;
            }
            lastTime_ = time;
            return lastValue_;
        }

        template<typename T>
        void StateSpaceFilter<T>::resetState() {
            lastTime_ = std::numeric_limits<double>::infinity();
            lastValue_ = 0.0;
            lastFirstDerivative_ = 0.0;
            lastSecondDerivative_ = 0.0;
        }

        template<typename T>
        void StateSpaceFilter<T>::resetState(double lastTime, double lastValue, double lastFirstDerivative, double lastSecondDerivative) {
            lastTime_ = std::numeric_limits<double>::infinity();
            lastValue_ = 0.0;
            lastFirstDerivative_ = 0.0;
            lastSecondDerivative_ = 0.0;
        }

        template<typename T>
        std::vector<T> StateSpaceFilter<T>::pass(const std::vector<T>& values, double f0) {

            resetState();
            std::vector<T> filteredValues;
            double time(0.), timeDelta(0.001);
            for (auto val : values) {
                filteredValues.push_back(filter(val, time, f0));
                time += timeDelta;
            }
            return filteredValues;
        }

        template<typename T>
        std::ostream & operator<< (std::ostream& os, const StateSpaceFilter<T>& filter) {
            os << "Current time : " << filter.lastTime_ << std::endl;
            os << "Current value : " << filter.lasValue_ << std::endl;
            os << "Current first derivative : " << filter.lastFirstDerivative_ << std::endl;
            os << "Current second derivative : " << filter.lastSecondDerivative_ << std::endl;

            return os;
        }
    }
} // namespace
