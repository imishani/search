#   * Copyright (c) 2024, Ramkumar Natarajan
#   * All rights reserved.
#   *
#   * Redistribution and use in source and binary forms, with or without
#   * modification, are permitted provided that the following conditions are met:
#   *
#   *     * Redistributions of source code must retain the above copyright
#   *       notice, this list of conditions and the following disclaimer.
#   *     * Redistributions in binary form must reproduce the above copyright
#   *       notice, this list of conditions and the following disclaimer in the
#   *       documentation and/or other materials provided with the distribution.
#   *     * Neither the name of the Carnegie Mellon University nor the names of its
#   *       contributors may be used to endorse or promote products derived from
#   *       this software without specific prior written permission.
#   *
#   * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#   * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#   * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#   * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#   * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#   * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#   * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   * POSSIBILITY OF SUCH DAMAGE.
#

sample_dict = {'g': -1, 'h': -1, 'f': -1, 'state': []}
data = []

# Open the file for reading
with open("../data/phai/0.txt", "r") as file:
    # Read each line from the file
    planner_name = file.readline().strip()
    map_id = file.readline().strip()
    start = list(map(float, file.readline().strip().split()))
    goal = list(map(float, file.readline().strip().split()))
    epsilon = float(file.readline().strip())
    for line in file:
        d = line.strip().split(' ')
        sample_dict['g'] = float(d[0])
        sample_dict['h'] = float(d[1])
        sample_dict['f'] = float(d[2])
        state = [float(val) for val in d[3:]]
        sample_dict['state'] = state
        data.append(sample_dict)

# Output the read data
print("Planner Name:", planner_name)
print("Map ID:", map_id)
print("Start:", start)
print("Goal:", goal)
print("Epsilon:", epsilon)
print("state data")
for sd in data:
    print(sd)

