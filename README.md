# acceleration_examples

[![](https://img.shields.io/badge/hardware_acceleration-KV260-ec1c24.svg)](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html)
[![](https://img.shields.io/badge/hardware_acceleration-ZCU102-ec1c24.svg)](https://www.xilinx.com/products/boards-and-kits/ek-u1-zcu102-g.html)


`acceleration_examples` is a meta-package that contain various package examples demonstrating the use of hardware acceleration in ROS 2. Each one of these examples aims to support all hardware acceleration technology solutions complying with REP-2008 (see [pending PR](https://github.com/ros-infrastructure/rep/pull/324)). By doing so, `acceleration_examples` aims to a) illustrate ROS package maintainers and ROS users how to build their own acceleration kernels and b) guarantee interoperability across technologies complying with [REP-2008](https://github.com/ros-infrastructure/rep/pull/324).

In turn, a CI system will be set to build the meta-package against all suported hardware.

- [acceleration_examples](#acceleration_examples)
    - [Quick access](#quick-access)
    - [ROS 2 Node acceleration examples](#ros-2-node-acceleration-examples)
    - [ROS 2 Graph acceleration examples](#ros-2-graph-acceleration-examples)
    - [Quality Declaration](#quality-declaration)

### Quick access

|   |    |     |
|---|----|-----|
| **ROS 2 Node acceleration** | | |
| [`publisher_xilinx`](nodes/publisher_xilinx) | [`simple_adder`](nodes/simple_adder) | [`vadd_publisher`](nodes/vadd_publisher) |  
| [`doublevadd_publisher`](nodes/doublevadd_publisher) | [`offloaded_doublevadd_publisher`](offloaded_doublevadd_publisher) | [`accelerated_vadd_publisher`](nodes/accelerated_doublevadd_publisher) |
| [`faster_doublevadd_publisher`](nodes/faster_doublevadd_publisher) | [`multiple_doublevadd_publisher`](nodes/multiple_doublevadd_publisher) | [`streaming_k2k_mm_xrt`](nodes/xrt_examples/streaming_k2k_mm_xrt) |
| [`image_proc_adaptive`](nodes/image_proc_adaptive) | [`image_pipeline_examples`](nodes/image_pipeline_examples) | |
| **ROS 2 Graph acceleration** | | |


### ROS 2 Node acceleration examples

<!-- 
| Package | Kernel     | Description |   Acceleration factor | Technology | CPU baseline | Accelerated  | -->
| Package | Acceleration kernel     | Description |
|---------|------------|-------------|
|[`publisher_xilinx`](nodes/publisher_xilinx) | | This package contains a minimalistic publisher using a member function for evaluation purposes which subclasses `rclcpp::Node` and sets up an `rclcpp::timer` to periodically call functions which publish messages. |   |  |  |
|[`simple_adder`](nodes/simple_adder) | [`adder1`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/simple_adder/src/adder1.cpp) [`adder2`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/simple_adder/src/adder2.cpp) | A trivial adder example. No interactions with the ROS 2 computational graph. Meant to demonstrate how HLS is integrated into build ROS 2 flows. |  **N/A** | KV260 | N/A |  |
|[`vadd_publisher`](nodes/vadd_publisher) | | A a trivial vector-add ROS 2 publisher. Adds two inputs to a vector in a loop and tries publishing it on the go at 10 Hz. |  | KV260 | 10 Hz<sup>[1](#myfootnote1)</sup> | |
|[`doublevadd_publisher`](nodes/doublevadd_publisher) | | A trivial double vector-add ROS 2 publisher. Adds two inputs to a vector in a loop and publishes on the go at 10 Hz. Running in hardware shows that it's not able to meet the rate target and stays at around 2 Hz. The objective of this package is to generate a computationally expensive baseline when executed in a general purpose embedded CPU. See [accelerated_doublevadd_publisher](nodes/accelerated_doublevadd_publisher) package for an optimized and accelerated version of the same package which offloads the vector operations into an FPGA. See [faster_doublevadd_publisher](nodes/faster_doublevadd_publisher) for an even more optimized version. |  | KV260 | 2 Hz<sup>[1](#myfootnote1)</sup> | |
| [`offloaded_doublevadd_publisher`](offloaded_doublevadd_publisher) | [`vadd`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/offloaded_doublevadd_publisher/src/vadd.cpp)  |  An offloaded version of the trivial [doublevadd_publisher](nodes/doublevadd_publisher) which adds two inputs to a vector in a loop and publishes them at 10 Hz. Vector add operations are directly offloaded into to the FPGA (i.e. a kernel is created out of C++, without any modifications). The offloading operation shows how while guaranteeing determinsm in the `vadd` operation context, simple offloading to the FPGA lowers the publishing rate from 2 Hz (in the CPU) to 1.5 Hz due the slower clock that the FPGA uses |      **0.75x**    |  KV260 |  2 Hz<sup>[1](#myfootnote1)</sup>  | 1.5 Hz<sup>[1](#myfootnote1)</sup> |
| [`accelerated_vadd_publisher`](nodes/accelerated_doublevadd_publisher) | [`vadd`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/accelerated_doublevadd_publisher/src/vadd.cpp)  |  An dataflow optimized offloaded version of the trivial [doublevadd_publisher](nodes/doublevadd_publisher) ROS 2 publisher which adds two inputs to a vector in a loop and publishes them at 10 Hz. Vector add operations are offloaded into to the FPGA and some minor dataflow optimizations are applied using HLS. The offloading operation into the FPGA allows the publisher to go from 2 Hz to 6 Hz but, still misses its target (10 Hz)  |      **3x**    |  KV260 |  2 Hz<sup>[1](#myfootnote1)</sup>  | 6 Hz<sup>[1](#myfootnote1)</sup> |
| [`faster_doublevadd_publisher`](nodes/faster_doublevadd_publisher) | [`vadd`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/faster_doublevadd_publisher/src/vadd.cpp)  |  An accelerated version of the trivial [doublevadd_publisher](nodes/doublevadd_publisher) ROS 2 publisher which adds two inputs to a vector in a loop and publishes them at 10 Hz. Vector add operations are accelerated by exploiting parallelism with the FPGA. Also, similarly to [accelerated_doublevadd_publisher](nodes/accelerated_doublevadd_publisher), some basic dataflow optimizations are performed. The code parallelism and dataflow optimizations of the dataflow allows the publisher to go from 2 Hz to 10 Hz, meeting its target |      **5x**    |  KV260 |  2 Hz<sup>[1](#myfootnote1)</sup> | 10 Hz<sup>[1](#myfootnote1)</sup> |
| [`multiple_doublevadd_publisher`](nodes/multiple_doublevadd_publisher) | [`vadd_offloaded`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/multiple_doublevadd_publisher/src/vadd_offloaded.cpp), [`vadd_accelerated`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/multiple_doublevadd_publisher/src/vadd_accelerated.cpp), [`vadd_faster`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/multiple_doublevadd_publisher/src/vadd_faster.cpp) | Smashes various acceleration kernels (coming from the previous *doublevadd_publisher series*) into a single hardware design (bitstream) that can then be used to program the FPGA, allowing to run various Nodes and their kernels simultaneously, without the need to reprogramm the FPGA. |
| [`streaming_k2k_mm_xrt`](nodes/xrt_examples/streaming_k2k_mm_xrt) | [`krnl_stream_vadd`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/xrt_examples/streaming_k2k_mm_xrt/src/krnl_stream_vadd.cpp), [`krnl_stream_vmult`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/xrt_examples/streaming_k2k_mm_xrt/src/krnl_stream_vmult.cpp) | This example demonstrates how kernels can have memory mapped   inputs along with stream interface from one kernel to another. Specifically, this is aROS native version of a simple kernel to kernelstreaming Vector Add and Vector Multiply C Kernel design with2 memory mapped input to kernel 1, 1 Stream output from kernel1 to input of kernel 2, 1 memory mapped input to kernel 2, and1 memory mapped output that demonstrates on how to process astream of data for computation between two kernels using XRTNative APIs. This design also illustrates how to set FIFO depthfor AXIS connections i.e. for the stream connecting the two kernels. |
| [`image_proc_adaptive`](nodes/image_proc_adaptive) | | An example that makes use of the ROS perception's [`image_pipeline`](https://github.com/ros-acceleration/image_pipeline)<sup>[3](#myfootnote3)</sup> meta-package to demonstrate  how to make a ROS 2 adaptive *Components* that can run either in CPU or in the FPGA. Refer to [`adaptive_component`](https://github.com/ros-acceleration/adaptive_component) for more details on a composable container for Adaptive ROS 2 Node computations.|
| [`image_pipeline_examples`](nodes/image_pipeline_examples) | | Provides Node wrappers to group together and demonstrate various combinations of hardware-accelerated perception pipelines using the [`image_pipeline`](https://github.com/ros-acceleration/image_pipeline)<sup>[3](#myfootnote3)</sup> package. |



- <a name="myfootnote1">1</a>: measured with `ros2 topic hz <topic-name>`
- <a name="myfootnote2">2</a>: very low rate.
- <a name="myfootnote3">3</a>: we're using [our fork](https://github.com/ros-acceleration/image_pipeline). In time, we expect these changes to be merged upstream.

### ROS 2 Graph acceleration examples
| Package | Acceleration kernel     | Description |
|---------|------------|-------------|
|[`publisher_xilinx`](publisher_xilinx) | | This package contains a minimalistic publisher using a member function for evaluation purposes which subclasses `rclcpp::Node` and sets up an `rclcpp::timer` to periodically call functions which publish messages. |   |  |  |



### Quality Declaration

No quality is claimed according to [REP-2004](https://www.ros.org/reps/rep-2004.html). This meta-package is designed as a learning and reference resource and it should not be used in production environments.