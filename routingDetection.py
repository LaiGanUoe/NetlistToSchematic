import matplotlib.pyplot as plt

# Define the necessary functions
def cross_product(o, a, b):
    # Calculate the cross product (a - o) x (b - o)
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def is_intersect(p1, p2, q1, q2):
    # Check if line segments P1P2 and Q1Q2 intersect
    d1 = cross_product(p1, p2, q1)
    d2 = cross_product(p1, p2, q2)
    d3 = cross_product(q1, q2, p1)
    d4 = cross_product(q1, q2, p2)

    if d1 * d2 < 0 and d3 * d4 < 0:
        return True  # The two segments intersect
    return False

def on_segment(p, q, r):
    # Check if point q lies between points p and r
    if min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]):
        return True
    return False

def is_overlap(p1, p2, q1, q2):
    # Calculate the cross product and check if the segments are collinear
    if cross_product(p1, p2, q1) == 0 and cross_product(p1, p2, q2) == 0:
        # Check if the line segments overlap
        if on_segment(p1, q1, p2) or on_segment(p1, q2, p2) or on_segment(q1, p1, q2) or on_segment(q1, p2, q2):
            return True  # The segments overlap
    return False

# Create the plot and draw the line segments
fig, ax = plt.subplots()

# Example 1: Non-intersecting line segments
# Modify the positions to ensure they don't intersect
p1, p2 = (1, 3), (10, 3)  # Blue segment at y = 3
q1, q2 = (12, 2), (18, 2)  # Red segment at y = 2
intersect_result = is_intersect(p1, p2, q1, q2)
ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', label="Line P1P2 (Not intersecting)")
ax.plot([q1[0], q2[0]], [q1[1], q2[1]], 'r-', label="Line Q1Q2 (Not intersecting)")

# Example 2: Intersecting line segments
p3, p4 = (1, 2), (10, 2)
q3, q4 = (5, 0), (5, 3)
intersect_result_2 = is_intersect(p3, p4, q3, q4)
ax.plot([p3[0], p4[0]], [p3[1], p4[1]], 'g-', label="Line P3P4 (Intersecting)")
ax.plot([q3[0], q4[0]], [q3[1], q4[1]], 'orange', label="Line Q3Q4 (Intersecting)")

# Example 3: Overlapping line segments
p5, p6 = (1, 1), (10, 1)
q5, q6 = (6, 1), (15, 1)
overlap_result = is_overlap(p5, p6, q5, q6)
ax.plot([p5[0], p6[0]], [p5[1], p6[1]], 'purple', label="Line P5P6 (Overlapping)")
ax.plot([q5[0], q6[0]], [q5[1], q6[1]], 'brown', label="Line Q5Q6 (Overlapping)")

# Set up the legend and title
ax.legend()
ax.set_title("Visualization of Line Segments: Non-intersecting, Intersecting, and Overlapping")
ax.set_xlim(0, 20)
ax.set_ylim(0, 5)  # Increase y-axis range to ensure all segments are visible

# Print results
print(f"Intersect Result (Example 1 ): {intersect_result}")
print(f"Intersect Result (Example 2 ): {intersect_result_2}")
print(f"Overlap   Result (Example 3 ): {overlap_result}")

# Display the plot
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.grid(True)
plt.show()