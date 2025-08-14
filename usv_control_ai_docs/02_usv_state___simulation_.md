# Chapter 2: USV State & Simulation

In the [previous chapter](01_mission___waypoint_management_.md), we learned how our USV gets its "itinerary"—a list of waypoints to visit. The Mission Manager acts like a tour guide, deciding which waypoint is next. But for the guide to know if they've reached the destination, they constantly need to ask: "Where are we *right now*?"

This chapter is all about answering that fundamental question. We'll explore how the USV keeps track of its own position, orientation, and speed, and how we can create a "virtual boat" to test our entire system without ever getting our feet wet.

### The Video Game Analogy

Think of a character in a video game. The game's software always knows your character's exact coordinates (x, y), which way they are facing, and how fast they're moving. This is the character's **state**.

When you press the "forward" button, the game's **physics engine** calculates the character's new state for the next frame. Our USV system works in a very similar way:

*   **USV State:** The boat's position, orientation (its "pose"), and velocity.
*   **The Simulator:** A "physics engine" for our boat. It takes commands (like "turn on thrusters") and calculates where the boat will be a fraction of a second later.

This simulator is a **digital twin** of the real USV. It allows us to run and test everything on a computer, which is much faster, cheaper, and safer than using a physical boat for every single code change.

## What is a "State"?

In our project, the USV's state is primarily made up of two key pieces of information:

1.  **Pose (`geometry_msgs/msg/Pose2D`)**: This tells us where the boat is and which way it's pointing on a 2D map. It consists of:
    *   `x`: The boat's horizontal position.
    *   `y`: The boat's vertical position.
    *   `theta`: The boat's rotation or heading, in radians.

2.  **Velocity (`geometry_msgs/msg/Vector3`)**: This describes how the boat is moving. For a surface vehicle, we care about:
    *   `x` (Surge): Forward/backward speed.
    *   `y` (Sway): Sideways speed (often near zero).
    *   `z` (Yaw Rate): How fast it's turning.

## The Simulator: Our Virtual Physics Engine

The core of our simulation is the `dynamic_model_sim` node. This program is a mathematical model of the USV's physical behavior. It listens for thruster commands and uses them to update the boat's state.

Let's see how it works.

### 1. Receiving Thruster Commands

The simulator needs to know how much power to apply to its virtual thrusters. It gets this information by listening to two topics: `/usv/left_thruster` and `/usv/right_thruster`.

```cpp
// File: src/dynamic_model_sim.cpp

// Listen for the left thruster command
leftThrusterSub = this->create_subscription<std_msgs::msg::Float64>(
    "usv/left_thruster", 10,
    [this](const std_msgs::msg::Float64 &msg) { 
        this->Tport = msg.data; // Tport is the force for the left (port) thruster
    });

// Listen for the right thruster command
rightThrusterSub = this->create_subscription<std_msgs::msg::Float64>(
    "usv/right_thruster", 10,
    // ... similar code to save the right thruster force ...
    );
```

This code tells the simulator: "Whenever a new power level for a thruster is sent, save that value." These values will be used in the next physics calculation.

### 2. Updating the State

Every 10 milliseconds, a function called `update()` runs. This is the heart of the physics engine.

```cpp
// File: src/dynamic_model_sim.cpp

void DynamicModelSim::update() {
    // 1. Run complex physics calculations based on thruster forces
    auto out = model.update(Tport, Tstbd);

    // 2. Extract the new pose (x, y, theta) from the results
    geometry_msgs::msg::Pose2D pose;
    pose.x = out.pose_x;
    pose.y = out.pose_y;
    pose.theta = normalize_angle(out.pose_psi);

    // 3. Publish the new pose for other nodes to use
    posePub->publish(pose);

    // 4. Also publish the new velocity
    // ... similar code to publish velocity ...
}
```

You don't need to understand the math inside `model.update()`. Just know that it takes the thruster forces (`Tport`, `Tstbd`) and simulates water resistance, momentum, and other forces to figure out the boat's new pose and velocity. It then **publishes** this new state on the `/usv/state/pose` topic.

## Broadcasting Our Position to the World (TF)

Publishing the state to a topic is great, but how do other parts of the system easily know where the boat is in relation to, say, the "world" or the map's origin?

For this, we use a core ROS 2 system called **TF (Transform)**. TF manages a tree of coordinate frames. Think of it as a universal GPS system for all components in our project.

The `usv_tf2_broadcaster_node` has one simple job: listen for the USV's pose and broadcast it to the TF system.

```mermaid
sequenceDiagram
    participant Controller
    participant Simulator
    participant State Topic </br> (/usv/state/pose)
    participant TF Broadcaster
    participant System (TF)

    Controller->>Simulator: Thruster Commands
    loop Every 10ms
        Simulator->>Simulator: Calculate new Pose
        Simulator->>State Topic: Publish Pose(x,y,θ)
    end
    TF Broadcaster->>State Topic: Subscribes to Pose
    TF Broadcaster->>System (TF): Broadcast Transform </br> "The USV is at (x,y,θ) relative to the world"
```

Here's the code that does the broadcasting:

```cpp
// File: src/usv_tf2_broadcaster_node.cpp

// This function is called whenever a new pose is published
void FramePublisher::handle_usv_pose(const std::shared_ptr<geometry_msgs::msg::Pose2D> msg)
{
  geometry_msgs::msg::TransformStamped t;

  // Set up the transform header
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";      // We are located relative to the world
  t.child_frame_id = usv_name_.c_str(); // The object being located is the USV

  // Copy the position and orientation from the message
  t.transform.translation.x = msg->x;
  t.transform.translation.y = msg->y;
  // ... code to set rotation from msg->theta ...

  // Send the transformation out to the whole system!
  tf_broadcaster_->sendTransform(t);
}
```

By broadcasting this transform, any other node (like the Mission Manager or a visualization tool) can simply ask the TF system, "Where is the `usv`?" and get an immediate, up-to-date answer. This is how the [Mission & Waypoint Management](01_mission___waypoint_management_.md) module knows if it's getting close to its target waypoint.

## Conclusion

You now know how our USV understands its own state! The **simulator** acts as a digital twin, using physics to calculate the boat's **pose** (position and orientation) and **velocity** based on thruster commands. This state is then published and broadcasted using **TF**, creating a reliable "You Are Here" pin that the entire autonomous system can use to make decisions.

With a mission in hand (Chapter 1) and a clear sense of our current location (Chapter 2), we can now tackle the next logical step: how do we draw the line on the map connecting where we are to where we want to go?

Next up: [Path Generation & Management](03_path_generation___management_.md)

---

Generated by [AI Codebase Knowledge Builder](https://github.com/The-Pocket/Tutorial-Codebase-Knowledge)