# Chapter 2: Segmentation

In the [previous chapter](01_point_cloud_filtering_.md), we learned how to tidy up our robot's 3D vision by filtering out noise and irrelevant data. We took a messy, raw point cloud and produced a clean one containing just the objects in our immediate workspace—a soda can sitting on a desk.

Our point cloud is now much cleaner, but it's still just one big blob of points. The robot doesn't know that some points belong to the desk and others belong to the can. To understand the scene, it needs to sort the points into meaningful groups. This sorting process is called **segmentation**.

### What Problem Does Segmentation Solve?

Segmentation is the process of partitioning a point cloud into multiple, distinct groups. It’s like sorting a bag of mixed vegetables into separate piles of carrots, peas, and corn. This is a fundamental step for scene understanding, as it breaks down a complex, unorganized cloud into simpler, meaningful parts like objects and surfaces.

**Use Case:** We have a clean point cloud of our desk and soda can. Our goal is to isolate the soda can so the robot can eventually pick it up. To do this, we need to teach the robot to distinguish between "table points" and "can points."


*Left: A filtered point cloud containing both a table and an object. Right: A segmented cloud where the table points (blue) are separated from the object points (red).*

Segmentation helps us:
1.  **Identify large surfaces:** We can find all the points that belong to a flat plane, like the tabletop.
2.  **Isolate objects:** We can group the remaining points into distinct clusters, like the soda can.
3.  **Prepare for recognition:** Once we have an isolated cluster for the can, we can then try to figure out *what* it is (e.g., "is this a can or a box?").

By the end of this chapter, you'll know how to build a simple perception pipeline to find a table and isolate the objects resting on it.

### Common Types of Segmentation

Just like we had different tools for filtering, we have different tools for segmenting. Let's look at two of the most common methods provided by `perception_pcl`.

#### 1. Model-Based Segmentation (`SACSegmentation`)

This method is perfect for finding simple geometric shapes within a point cloud. You tell it what kind of shape to look for (a plane, a cylinder, a sphere), and it finds the biggest and best group of points that fit that shape. It uses an algorithm called **RANSAC (Random Sample Consensus)**.

*   **Analogy:** Imagine throwing a handful of marbles onto a wooden floor. Some marbles might bounce into corners, but most will lie on the flat floor. RANSAC is like trying to find the "floor" by repeatedly picking three random marbles, assuming they form a flat plane, and then counting how many other marbles lie on that same plane. After many tries, you'll find the plane that has the most marbles on it—that's your floor!

We will use `SACSegmentation` to find all the points that belong to our flat tabletop.

#### 2. Proximity-Based Segmentation (`EuclideanClusterExtraction`)

After we find and remove the table, we're left with the points for the soda can. But what if there were two cans on the table? This method helps us group points that are physically close to each other into separate clusters.

*   **Analogy:** Think of a crowd of people at a party. Some people are gathered in small conversation circles. Euclidean Cluster Extraction is like starting with one person, finding everyone within arm's reach, then finding everyone within arm's reach of *them*, and so on, until you've identified one whole conversation circle. You then move to a person not in that circle and repeat the process to find the next group.

We will use `EuclideanClusterExtraction` to group the points for the soda can into a single, isolated object.

### Building a Segmentation Pipeline in ROS 2

Let's chain these tools together to solve our use case. We'll perform a two-step process: first, find and remove the table plane, and second, cluster the remaining points.

#### Step 1: Find the Table Plane

We'll use the `sac_segmentation` node to find the points that belong to the table. This node subscribes to a point cloud and publishes two things:
1.  `inliers`: A list of indices of the points that fit the model (the table points).
2.  `model`: The mathematical description of the shape it found (e.g., the equation of the plane).

```bash
ros2 run pcl_ros sac_segmentation_node --ros-args \
   -r input:=/my_robot/filtered_points \
   -p model_type:=0 \
   -p distance_threshold:=0.01
```

*   `input:=/my_robot/filtered_points`: The node listens to the clean point cloud from our filtering step.
*   `p model_type:=0`: We tell the node to look for a plane (`SACMODEL_PLANE` has a value of 0).
*   `p distance_threshold:=0.01`: This is our RANSAC tolerance. Any point within 1cm of the candidate plane will be considered part of it.

After running this, the node will be publishing the indices of all the table points on the `/inliers` topic.

#### Step 2: Isolate and Cluster the Can

Now that we know which points are the table, we can ignore them and focus on what's left. We can use another node, `ExtractIndices`, to do this subtraction. Then, we feed the remaining points (just the can) into the `euclidean_cluster_extraction` node.

(Note: For simplicity, we'll just show the final clustering step here.)

The `euclidean_cluster_extraction` node takes the points that are *not* the table and groups them into objects.

```bash
ros2 run pcl_ros euclidean_cluster_extraction_node --ros-args \
    -r input:=/points_without_plane \
    -p cluster_tolerance:=0.02 \
    -p min_cluster_size:=100
```

*   `input:=/points_without_plane`: This node would listen to the cloud after the table has been removed.
*   `p cluster_tolerance:=0.02`: If two points are closer than 2cm, they might belong to the same object.
*   `p min_cluster_size:=100`: An object must have at least 100 points to be considered valid. This prevents tiny noise clusters from being treated as objects.

The output of this node is one or more new point cloud topics, each containing a single detected object! Our robot has now successfully isolated the soda can.

### Under the Hood: How a Segmentation Node Works

Let's look inside the `sac_segmentation` node to see how it finds the plane. The process is very similar to the filter node from the last chapter.

```mermaid
sequenceDiagram
    participant InputCloud as Input Topic (/filtered_points)
    participant SegNode as SACSegmentation Node
    participant PCL as PCL Library
    participant Inliers as Output Topic (/inliers)
    participant Model as Output Topic (/model)

    InputCloud->>+SegNode: Publishes PointCloud2 message
    SegNode->>PCL: Converts ROS message to PCL format
    Note right of SegNode: The node prepares the data for PCL.
    SegNode->>PCL: Runs RANSAC segmentation algorithm
    PCL-->>-SegNode: Returns inlier indices & plane model
    SegNode->>Inliers: Publishes inlier PointIndices message
    SegNode->>Model: Publishes ModelCoefficients message
    Inliers-->>-SegNode:
    Model-->>-SegNode:
```

The node orchestrates the process: it gets data from ROS, calls the powerful PCL library to do the hard math, and then publishes the results back into the ROS ecosystem.

Let's examine some simplified code from the project.

#### 1. Setting Up Parameters

In its `onInit` function, the node declares the parameters it needs. This is how we can specify the `model_type` and `distance_threshold` from the command line.

(from `pcl_ros/src/pcl_ros/segmentation/sac_segmentation.cpp`)
```cpp
// In the node's onInit() function...
int model_type;
// This tells the node to get the 'model_type' parameter.
// If it's not found, it will report an error.
if (!pnh_->getParam("model_type", model_type)) {
  NODELET_ERROR("Need a 'model_type' parameter...");
  return;
}

// Set the model type for the underlying PCL implementation.
impl_.setModelType(model_type);
```
This code connects the ROS parameter system to the internal PCL algorithm's configuration.

#### 2. The Core Segmentation Logic

The `input_indices_callback` function is where the main work happens every time a new point cloud arrives.

(from `pcl_ros/src/pcl_ros/segmentation/sac_segmentation.cpp`)
```cpp
void pcl_ros::SACSegmentation::input_indices_callback(
  const PointCloudConstPtr & cloud, ...)
{
  // ... (Input validity checks are done here) ...

  // Set the input cloud for the PCL algorithm.
  impl_.setInputCloud(pcl_ptr(cloud));

  // Create PCL objects to store the results.
  pcl::PointIndices pcl_inliers;
  pcl::ModelCoefficients pcl_model;

  // The magic happens here! PCL runs the segmentation.
  impl_.segment(pcl_inliers, pcl_model);

  // ... (Convert results from PCL format back to ROS messages) ...

  // Publish the final ROS messages.
  pub_indices_.publish(inliers);
  pub_model_.publish(model);
}
```
Just like with filtering, the node's main job is data management. It takes a ROS cloud, passes it to the PCL implementation (`impl_`), and calls the `.segment()` method. This single line triggers the complex RANSAC algorithm, which finds the best plane in the data and returns the results.

### Conclusion

You've now learned how to take a cleaned-up point cloud and make sense of its structure. Segmentation is the crucial step that transforms a simple collection of points into a set of distinct surfaces and objects.

We learned about:
-   **Why segmentation is necessary:** To group points into meaningful parts of the scene.
-   **Common segmentation types:** `SACSegmentation` for finding geometric models and `EuclideanClusterExtraction` for grouping by proximity.
-   **How to build a pipeline in ROS 2:** By chaining nodes together to first find and remove a plane, then cluster the remaining objects.

We have successfully isolated the point cluster for our soda can. But what can the robot *do* with this cluster? How can it describe the can's properties, like its size or position? To answer that, we need to extract key information from the cluster. This process is called **feature extraction**, and it's the topic of our next chapter.

[Next: Feature Extraction](03_feature_extraction_.md)

---

Generated by [AI Codebase Knowledge Builder](https://github.com/The-Pocket/Tutorial-Codebase-Knowledge)