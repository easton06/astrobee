# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# The observation from a camera image, of image coordinates
# and associated 3D coordinates. Used for sparse mapping and AR tags.

Header header # header with timestamp
uint32 camera_id # image ID, associated with registration pulse
geometry_msgs/Pose pose # estimated camera pose from features
ff_msgs/VisualLandmark[] landmarks # list of all landmarks
float32 runtime # Time it took to calculate the pose and matching landmarks
