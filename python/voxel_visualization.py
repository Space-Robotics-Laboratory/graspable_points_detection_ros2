import pandas as pd
import plotly.express as px
#vector3d_cpp = pd.read_csv("/home/diane/Downloads/vector3D.csv", names=["x","y","z"])
# print(vector3d_cpp.head())
#vector3d_matlab = pd.read_csv("/home/j02-scare/Documents/Results/Vector3D_matlab.csv", names=["x","y","z"])



vector3d_gripper_cpp = pd.read_csv("/home/srl-limb/ros2_ws/src/SRL_GraspableTargetDetection/detect_graspable_points/pcd_data/GripperMask.csv", names=["x","y","z"])


#vector3d_gripper_cpp = pd.read_csv("/home/jitao/Documents/Results/Voxel_array/GripperMask_matlab_2.csv", names=["x","y","z"])
#graspable_points = pd.read_csv("/home/j02-scare/Documents/Results/Voxel_coordinates_of_graspable_points.csv", names=["x","y","z"])
print(vector3d_gripper_cpp.head())
vector3d_gripper_cpp.drop_duplicates(inplace=True)
print(vector3d_gripper_cpp.shape[0])
def visualize_3d(vector_3d, size_mask=1):
    fig = px.scatter_3d(vector_3d, x='x', y='y', z='z')
    fig.update_scenes(aspectmode='data')
    fig.update_traces(marker=dict(size=10,
    				symbol="square",
                                line=dict(width=2,
                                            color='DarkSlateGrey')),
                    selector=dict(mode='markers'))
    fig.show()
    print(vector3d_gripper_cpp.size)
# visualize_3d(vector3d_cpp)
# visualize_3d(vector3d_matlab)
visualize_3d(vector3d_gripper_cpp,5)
# visualize_3d(vector3d_gripper_matlab,5)
#visualize_3d(graspable_points)
