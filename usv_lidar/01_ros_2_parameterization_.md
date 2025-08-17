# Chapter 1: ROS 2 Parameterization

Welcome to the `usv_lidar` project tutorial! In this first chapter, we're going to explore a fundamental and powerful concept in ROS 2: **Parameterization**.

### Why Do We Need Parameters?

Imagine you've built an amazing LiDAR-based object detection system for your Unmanned Surface Vehicle (USV). It works, but you notice it's a bit too sensitive. It keeps detecting the small ripples on the water's surface as obstacles. How would you fix this?

You could go back into your C++ code, find the line that controls sensitivity, change the value from, say, `0.1` to `0.2`, and then recompile your entire project. This might take a few minutes. Then you test it and realize `0.2` was too much. So you change it to `0.15`, recompile again, and test again. This process is slow, tedious, and error-prone.

Wouldn't it be great if you could just have a simple "settings" file, like a settings menu in a video game? You could just open a text file, tweak the sensitivity value, and restart the program to see the new results instantly.

This is exactly what **ROS 2 Parameters** allow us to do! They let us configure our robot's behavior externally, without ever touching the source code and recompiling.

### The Three Key Pieces

The ROS 2 parameter system is built on three main components that work together:

1.  **The C++ Node (`.cpp`):** This is our main program. It needs to *declare* which variables it wants to make configurable. It's like telling ROS, "I have a setting called `cluster_tolerance` that others can change."
2.  **The Configuration File (`.yaml`):** This is a simple text file where we store the actual values for our settings. For example, it's where we would write `cluster_tolerance: 0.25`.
3.  **The Launch File (`.py`):** This is the script that starts our C++ node. It acts as the bridge, telling the node, "When you start, load your settings from this specific configuration file."

Let's see how these three pieces work together in our project.

### Step 1: Declaring a Parameter in C++

First, our C++ node needs to announce the parameters it's willing to accept. Inside the constructor of our `VoxelGrid_filter` class, you'll see several declarations.

**File:** `src/clustering_segmentation.cpp`
```cpp
/* Parameters */
this->declare_parameter("voxel_grid_x",rclcpp::PARAMETER_DOUBLE);
// ... more declarations ...
this->declare_parameter("cluster_tolerance",rclcpp::PARAMETER_DOUBLE);
this->declare_parameter("min_cluster",rclcpp::PARAMETER_INTEGER);
this->declare_parameter("max_cluster",rclcpp::PARAMETER_INTEGER);
```

This code tells the ROS 2 system:
- "I have a parameter named `cluster_tolerance`. It should be a floating-point number (a `DOUBLE`)."
- "I also have a parameter named `min_cluster`. It should be a whole number (an `INTEGER`)."

Think of this as creating empty slots or placeholders for our settings.

### Step 2: Setting Values in the YAML File

Next, we need to provide the actual values for these placeholders. We do this in a simple, human-readable file called `params.yaml`.

**File:** `config/params.yaml`
```yaml
clustering_segmentation:
  ros__parameters:
    # ... other parameters ...
    cluster_tolerance: 0.25
    min_cluster: 20
    max_cluster: 200
```
This file is very straightforward. It's structured like a list:
- `clustering_segmentation:` This is the name of our node.
- `ros__parameters:` This is a required keyword.
- `cluster_tolerance: 0.25`: This is our setting. We're giving the `cluster_tolerance` parameter a value of `0.25`.

### Step 3: Loading the Parameters with a Launch File

Now we have our node that expects parameters and a file that contains the values. The final step is to connect them. The launch file is the glue that holds it all together.

**File:** `launch/process_cloud.launch.py`
```python
def generate_launch_description():
    params_file_path = os.path.join(
        # ... finds the path to the file ...
        ,'config','params.yaml')

    clustering_segmentation_node = Node(
        package='usv_lidar',
        executable='clustering_segmentation',
        parameters=[params_file_path]) # <--- This is the magic!
    # ...
```
When we run this launch file, it starts the `clustering_segmentation` node. The key part is `parameters=[params_file_path]`. This tells the node, "Hey, look in the `params.yaml` file for your settings!"

### How It All Comes Together

Let's visualize the entire process from start to finish.

```mermaid
sequenceDiagram
    participant Launch File
    participant params.yaml
    participant ROS 2 System
    participant C++ Node

    Launch File->>C++ Node: Starts the node
    Launch File->>ROS 2 System: Tells it to use params.yaml for this node
    ROS 2 System->>params.yaml: Reads key-value pairs
    ROS 2 System->>C++ Node: Provides parameters (e.g., cluster_tolerance=0.25)
    C++ Node->>C++ Node: (Inside code) timer_callback is triggered
    C++ Node->>ROS 2 System: get_parameter("cluster_tolerance")
    ROS 2 System->>C++ Node: Returns 0.25
    C++ Node->>C++ Node: Uses value 0.25 in its algorithm
```

Inside the C++ code, when the node needs to use a parameter value, it simply asks for it.

**File:** `src/clustering_segmentation.cpp`
```cpp
// Get the parameter object from ROS
rclcpp::Parameter cluster_tolerance_param = this->get_parameter("cluster_tolerance");

// Convert it to a usable float variable
float cluster_tolerance = cluster_tolerance_param.as_double();

// Now use it in the algorithm!
ec.setClusterTolerance (cluster_tolerance);
```

This code fetches the value that was originally loaded from our `params.yaml` file and uses it to configure the clustering algorithm.

### Let's Tune Our Robot!

Now for the fun part. Let's go back to our original problem: our robot is detecting two nearby buoys as a single, large object. This is because the `cluster_tolerance` is too high. It's grouping points that are too far apart.

To fix this, we don't need to recompile anything. We just need to edit one line.

**1. Open the file:** `config/params.yaml`

**2. Change the value:**
```yaml
# Before
cluster_tolerance: 0.25

# After
cluster_tolerance: 0.1
```

**3. Save the file and restart the system** using the launch file.

That's it! The node will now start with the new, smaller value of `0.1`. It will be "less tolerant" and will only group points that are very close together, allowing it to correctly identify the two buoys as separate objects.

### Conclusion

In this chapter, we learned about the power of ROS 2 Parameters. They provide a clean and efficient way to separate our algorithm's logic from its configuration. This makes our system flexible and easy to tune.

We saw the three key components:
- **Declaring** parameters in C++.
- **Setting** their values in a `.yaml` file.
- **Loading** the file using a `.py` launch file.

This simple but powerful pattern is used throughout ROS 2. Now that you understand how to configure our node, we can begin to explore what it actually does.

In the next chapter, we'll look at the very first processing step our LiDAR data goes through.

[Next Chapter: VoxelGrid_filter ROS 2 Node](02_voxelgrid_filter_ros_2_node_.md)

---

Generated by [AI Codebase Knowledge Builder](https://github.com/The-Pocket/Tutorial-Codebase-Knowledge)