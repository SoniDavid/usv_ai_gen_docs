# Chapter 2: ROS2 Node Pipeline

In the [previous chapter](01_system_configuration_.md), we learned how to use the `params.yaml` file as a "Settings Menu" for LIO-SAM. We saw how to tell the system about our specific robot and sensors. Now, let's zoom out and look at the big picture. What exactly *is* the LIO-SAM system that we're configuring? How does it take raw sensor data and turn it into a map?

### The SLAM Assembly Line

Imagine a car factory. Building a car is a huge, complicated task. You don't have one person or one giant machine do everything at once. Instead, you have an **assembly line**.

1.  A bare metal frame starts at one end.
2.  The first station adds the engine.
3.  The next station attaches the wheels.
4.  Another adds the doors.
5.  ...and so on, until a finished car rolls out at the end.

Each station is a specialist. The "wheel station" only knows how to attach wheels, and it does it very well. It receives a car-without-wheels from the previous station and passes a car-with-wheels to the next.

LIO-SAM works in exactly the same way. It's a pipeline of specialized programs, called **ROS2 Nodes**, that act like stations on an assembly line. Each node has one specific job. They communicate by passing data along channels, called **ROS2 Topics**, which act like the conveyor belts of our factory.

This modular design is brilliant because it breaks the massive problem of SLAM into smaller, understandable, and manageable pieces.

### The Main "Stations" in LIO-SAM

LIO-SAM's assembly line consists of four main "workers" or nodes. Let's meet them:

1.  **`imageProjection`**: This is the first worker on the line. Its job is to take the raw, jumbled mess of points from the LiDAR and organize them into a neat grid, like a picture. It also corrects for any shaking or wobbling that happened while the scan was being taken. We'll learn more about this in [Sensor Data Preprocessing & Deskewing](04_sensor_data_preprocessing___deskewing_.md).
2.  **`featureExtraction`**: This worker receives the organized point cloud. Its job is to be a detective and find the "interesting" points. It looks for sharp edges and flat surfaces, which are much easier to track than a generic blob of points. This is covered in [LiDAR Feature Extraction](05_lidar_feature_extraction_.md).
3.  **`imuPreintegration`**: This worker runs on a separate, parallel assembly line. It processes the high-frequency data from the IMU to create a short-term prediction of the robot's motion. Think of it like feeling the motion of a car with your eyes closed. This is detailed in [IMU Preintegration](03_imu_preintegration_.md).
4.  **`mapOptimization`**: This is the final and most important station. It's the "master assembler." It takes the feature points, the IMU motion predictions, and the map built so far, and puts them all together. It figures out the most likely position of the robot and adds the new points to the global map. We explore this in [Mapping & Factor Graph Optimization](06_mapping___factor_graph_optimization_.md).

Here is a simplified diagram of this assembly line:

```mermaid
graph TD
    subgraph LIO-SAM Pipeline
        A[Raw LiDAR Data <br> (/points)] --> B(imageProjection Node);
        C[Raw IMU Data <br> (/imu/data)] --> D(imuPreintegration Node);
        B --> E(featureExtraction Node);
        D --> F(mapOptimization Node);
        E --> F;
        F --> G[Final Map & Pose <br> (Odometry)];
    end
```

### Under the Hood: The "Factory Manager" Launch File

So who tells all these workers to show up and start their jobs? That's the job of the **launch file**: `launch/run.launch.py`.

In Chapter 1, we saw that the launch file reads our `params.yaml` configuration. Its other main job is to start each of the ROS2 nodes. It's like the factory manager who flips the switch to turn on the entire assembly line.

Let's look at a tiny piece of `launch/run.launch.py`:

```python
# In launch/run.launch.py

# ... other setup code ...
Node(
    package='lio_sam',
    executable='lio_sam_imageProjection',
    parameters=[parameter_file],
    output='screen'
),
```

This block tells ROS2: "Please start a program."
-   `package='lio_sam'`: The program is part of the `lio_sam` project.
-   `executable='lio_sam_imageProjection'`: The specific program to run is `lio_sam_imageProjection`. This is our first "worker."
-   `parameters=[parameter_file]`: Give this worker all the settings from our `params.yaml` file.

The launch file has a similar `Node(...)` block for `featureExtraction`, `mapOptimization`, and `imuPreintegration`, starting up the entire pipeline.

### Under the Hood: The "Conveyor Belt" Topics

How does the `imageProjection` node hand its work over to the `featureExtraction` node? They use the ROS2 topic "conveyor belts."

One node **publishes** data to a topic, and another node **subscribes** to it.

In the C++ code for `imageProjection`, you'll find a line that creates a publisher. This is like setting up the output end of a conveyor belt.

```cpp
// In src/imageProjection.cpp
// Simplified for clarity

pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
    "lio_sam/deskew/cloud_info", qos);
```
This code says: "I am going to publish organized cloud information on the topic named `/lio_sam/deskew/cloud_info`."

Then, in the `featureExtraction` code, you'll see a line that creates a subscriber. This is the input end of the conveyor belt.

```cpp
// In src/featureExtraction.cpp
// Simplified for clarity

subLaserCloudInfo = create_subscription<lio_sam::msg::CloudInfo>(
    "lio_sam/deskew/cloud_info", qos, ...);
```
This code says: "I am listening for data on the topic named `/lio_sam/deskew/cloud_info`. When data arrives, I'll process it."

This publish-subscribe pattern is the fundamental communication method that connects all the nodes in the LIO-SAM pipeline. It's powerful because the `imageProjection` node doesn't need to know or care who is listening. It just does its job and puts the result on the conveyor belt. You could easily add a new, different node that also listens to that topic without changing the original node at all!

### Conclusion

You've now seen the high-level architecture of LIO-SAM. It isn't one giant, monolithic program. Instead, it's an elegant assembly line—a **pipeline of specialized ROS2 nodes**. Each node performs a distinct task and communicates with the others using ROS2 topics. This modular design makes the system easier to understand, debug, and maintain.

We've seen the "what" (the nodes) and the "how" (the topics and launch file). Now we're ready to walk down the assembly line and inspect each station individually. We'll start with the parallel process that handles the robot's motion in the next chapter: [IMU Preintegration](03_imu_preintegration_.md).

---

Generated by [AI Codebase Knowledge Builder](https://github.com/The-Pocket/Tutorial-Codebase-Knowledge)