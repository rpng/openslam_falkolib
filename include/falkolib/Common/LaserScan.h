/**
 * FALKOLib - Fast Adaptive Laser Keypoint Orientation-invariant
 * Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.
 *
 * This file is part of FALKOLib.
 *
 * FALKOLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * FALKOLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FALKOLib.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <vector>
#include <falkolib/Common/Point.h>
#include <falkolib/Common/GeomUtils.h>

namespace falkolib {

	/**
	 * @brief Laser scan container
	 * 
	 * This class provides an essential interface for laser scan data 
	 */
	class LaserScan {
	public:

		/**
		 * @brief Constructor
		 */
		LaserScan() {
			//angleMin = 0;
			//fov = 0;
			//angleInc = 0;
			//numBeams = 0;
			//timestamp = 0;
		}

		/**
		 * @brief Constructor for evenly spaced scans
		 * @param _angleMin laser scanner start angle [rad]
		 * @param _fov laser scanner field of view [rad]
		 * @param _ranges the range measurements of each scan point [meters]
		 */
		LaserScan(double _angleMin, double _fov, const std::vector<double>& _ranges) {
            ranges = _ranges;
			//angleMin = _angleMin;
			//fov = _fov;
			double angleInc = _fov / _ranges.size();
            angleIncAvg = angleInc;
			//numBeams = _numBeams;
			//timestamp = 0;
            points.resize(ranges.size());
            bearings.resize(ranges.size());
			for (size_t i = 0; i < ranges.size(); ++i) {
				double theta = i * angleInc + _angleMin;
                bearings[i] = theta;
				points[i][0] = ranges[i] * std::cos(theta);
				points[i][1] = ranges[i] * std::sin(theta);
			}
        
		}
		
        //LaserScan(double _angleMin, double _fov, const double* _ranges, int numBeams) {
        //    LaserScan(_angleMin, _fov, std::vector<double>(_ranges, _ranges + numBeams);
        //}
		
        /**
		 * @brief Constructor for unevenly spaced scans
		 * @param _angleMin laser scanner start angle [rad]
		 * @param _fov laser scanner field of view [rad]
		 * @param _ranges the range measurements of each scan point [meters]
		 */
		LaserScan(const std::vector<double>& _bearings, const std::vector<double>& _ranges) {
            assert(_bearings.size() == _ranges.size());
            ranges = _ranges;
            bearings = _bearings;
            points.resize(ranges.size());
            double angle_max = -1;
            double angle_min = -1;
			for (size_t i = 0; i < ranges.size(); ++i) {
				double theta = bearings[i];
                if (i > 0) {
                    angle_max = std::max(angle_max, theta);
                    angle_min = std::min(angle_min, theta);
                } else {
                    angle_max = angle_min = theta;
                }
				points[i][0] = ranges[i] * std::cos(theta);
				points[i][1] = ranges[i] * std::sin(theta);
			}
            angleIncAvg = (angle_max - angle_min) / (ranges.size() - 1);
		}

		/** @brief Set laser scanner start angle [rad] */
		//inline void setAngleMin(double _angleMin) {
		//	angleMin = _angleMin;
		//};

		/** @brief Set laser scanner field of view [rad] */
		//inline void setLaserFoV(double _fov) {
		//	fov = _fov;
		//};

		/** @brief Set laser scanner angle increment [rad] */
		//inline void setAngleInc(double _angleInc) {
		//	angleInc = _angleInc;
		//};

		/** @brief Set laser scanner number of beams */
		//inline void setNumBeams(int _numBeams) {
		//	numBeams = _numBeams;
		//};

		/** @brief Set scan beginning timestamp [s] */
		//inline void setTimestamp(double _timestamp) {
		//	timestamp = _timestamp;
		//};

		/** @brief Get laser scanner number of beams */
		inline int getNumBeams() const {
			return (int)ranges.size();
		};

		/** @brief Get laser scanner angle increment (or avg increment if unevenly spaced) [rad] */
		inline double getAngleInc() const {
		    return angleIncAvg;
		};

		/**
		 * @brief Compute scan points from ranges
		 * 
		 * @param _ranges plain array of double representing the scan ranges
		 */
		//inline void fromRanges(const double* _ranges) {
		//	fromRanges(std::vector<double>(_ranges, _ranges + numBeams));
		//}

		/**
		 * @brief Compute scan points from ranges
		 * 
		 * @param _ranges std::vector of double representing the scan ranges
		 */
		//inline void fromRanges(const std::vector<double>& _ranges) {
		//	double theta;
		//	ranges = _ranges;
		//	points.resize(numBeams);
		//	for (int i = 0; i < numBeams; ++i) {
		//		theta = i * angleInc + angleMin;
		//		points[i][0] = ranges[i] * std::cos(theta);
		//		points[i][1] = ranges[i] * std::sin(theta);
		//	}
		//}

		/**
		 * @brief compute neighborhood points list given a single point index and a search radius
		 * @param candIndex index of the central point
		 * @param radius search euclidean radius [m]
		 * @param neigh vector of the neighborhood points
		 * @param midIndex index representing the central point in the neigh vector
		 */
		inline void getNeighPoints(int candIndex, double radius, std::vector<Point2d>& neigh, int& midIndex) const {
			const Point2d& candPoint = points[candIndex];
			//int alpha = std::floor(std::asin(radius / ranges[candIndex]) / angleInc);
			int alpha = std::floor(std::asin(radius / ranges[candIndex]) / angleIncAvg);
			int begIndex = std::max(0, candIndex - alpha);
			int endIndex = std::min(candIndex + alpha + 1, (int)ranges.size());
			for (int i = begIndex; i <= endIndex; ++i) {
				if (pointsDistance(points[i], candPoint) <= radius) {
					if (i == candIndex) {
						midIndex = neigh.size();
					}
					neigh.push_back(points[i]);
				}
			}
		}

		std::vector<double> ranges;
		std::vector<double> bearings;
		std::vector<Point2d> points;

	protected:

		//double angleMin;
		//double fov;
		double angleIncAvg;
		//int numBeams;
		//double timestamp;
	};
}
