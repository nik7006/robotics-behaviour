import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import time

st.set_page_config(page_title="Autonomous Robot Navigation", layout="wide")

st.title("🤖 Autonomous Robot Navigation with Obstacles")

# Sidebar controls
st.sidebar.header("Settings")

# Number of obstacles
num_obs = st.sidebar.slider("Number of Obstacles", 1, 6, 3)

# Goal coordinates
goal_x = st.sidebar.number_input("Goal X", value=10.0)
goal_y = st.sidebar.number_input("Goal Y", value=10.0)

goal = np.array([goal_x, goal_y])

# Obstacle coordinates
st.sidebar.subheader("Obstacle Coordinates")

obstacles = []
for i in range(num_obs):
    x = st.sidebar.number_input(f"Obstacle {i+1} X", value=float(i+3), key=f"x{i}")
    y = st.sidebar.number_input(f"Obstacle {i+1} Y", value=float(i+4), key=f"y{i}")
    obstacles.append(np.array([x, y]))

# -----------------------------
# Reference Map
# -----------------------------

st.subheader("Environment Map")

fig_ref, ax_ref = plt.subplots(figsize=(6,6))

# Obstacles (only cross markers)
for obs in obstacles:
    ax_ref.scatter(obs[0], obs[1], marker='x', s=150, color="black")

# Goal
ax_ref.scatter(goal[0], goal[1], marker='*', s=300, color="green", label="Goal")

# Start
ax_ref.scatter(0, 0, color="red", s=120, label="Start")

ax_ref.set_xlim(-1, 12)
ax_ref.set_ylim(-1, 12)

ax_ref.grid(True)
ax_ref.legend()

st.pyplot(fig_ref)

# -----------------------------
# Robot Simulation
# -----------------------------

if st.button("🚀 Start Robot"):

    robot = np.array([0.0, 0.0])

    path_x = [robot[0]]
    path_y = [robot[1]]

    plot_area = st.empty()

    step = 0

    while np.linalg.norm(robot - goal) > 0.3:

        step += 1

        # Attractive force toward goal
        goal_force = goal - robot
        goal_force = 1.2 * (goal_force / np.linalg.norm(goal_force))

        # Repulsive force from obstacles
        repulsive = np.array([0.0, 0.0])

        for obs in obstacles:

            dist = np.linalg.norm(robot - obs)

            if dist < 2.0:

                direction = robot - obs
                direction = direction / np.linalg.norm(direction)

                repulsive += direction * (0.3 / (dist**2))

        move = goal_force + repulsive
        move = move / np.linalg.norm(move)

        robot = robot + move * 0.35

        path_x.append(robot[0])
        path_y.append(robot[1])

        # Plot simulation
        fig, ax = plt.subplots(figsize=(6,6))

        ax.plot(path_x, path_y, linewidth=2, label="Robot Path")

        ax.scatter(robot[0], robot[1], s=120, color="red", label="Robot")

        ax.scatter(goal[0], goal[1], marker='*', s=300, color="green", label="Goal")

        # Obstacles (only crosses)
        for obs in obstacles:
            ax.scatter(obs[0], obs[1], marker='x', s=150, color="black")

        ax.set_xlim(-1, 12)
        ax.set_ylim(-1, 12)

        ax.set_title(f"Step {step} | Distance: {np.linalg.norm(robot-goal):.2f}")

        ax.grid(True)
        ax.legend()

        plot_area.pyplot(fig)

        time.sleep(0.35)

    st.success("🎯 Goal Reached!")
