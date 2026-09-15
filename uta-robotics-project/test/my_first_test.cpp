/*
 * Copyright (c) 2020-2021 error-404
 *
 * This file is part of uta-robotics.
 *
 * uta-robotics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * uta-robotics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with uta-robotics.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "my_first_file.hpp"

TEST(custom, name_required) {EXPECT_EQ(67, sum2());}

