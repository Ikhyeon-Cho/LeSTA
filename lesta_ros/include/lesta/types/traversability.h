/*
 * traversability.h
 *
 *  Created on: Feb 16, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

namespace lesta_types {

enum class Traversability : int {
  TRAVERSABLE = 1,
  NON_TRAVERSABLE = 0,
  UNKNOWN = -1,
};
} // namespace lesta_types