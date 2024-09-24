  // ****** General Parameters ******* //
#include <cmath>

//choose a model WARNING: only put one to true or you wiill have conflict
#define LIMBERO true
#define SCARE false
#define HUBROBO false

  // MatchingSettings
#define VOXEL_SIZE 0.002 // voxel size [m]
#define THRESHOLD 100// threshold of numbers of solid voxels within the subset (TSV) (SCAR-E: 120)
#define ARTFICIALY_ADD_POINTS false // choose wether or not you want to add more points during interpolation process in case of sparse map
#define DELETE_LOWER_TARGETS_THRESHOLD 0.015 // [m] Lower threshold of targets
#define EXTRA_SHEET 3 // size of extra sheet above the top layer of gripper mask (H_add)(SCAR-E: 1)
#define GRASPABILITY_THRESHOLD 90 // [%] Graspability threshold. Above which we can call it graspable with great certainty


// gripper parameters in [mm]!
//Limbero
#if LIMBERO
#define PALM_DIAMETER 62 // PALM_DIAMETER
#define PALM_DIAMETER_OF_FINGER_JOINTS 69 // 
#define FINGER_LENGTH 26 // FINGER_LENGTH
#define SPINE_LENGTH 25 // SPINE_LENGTH
#define SPINE_DEPTH 12 // SPINE_DEPTH
#define OPENING_ANGLE 79 // OPENING_ANGLE
#define CLOSING_ANGLE 42 // CLOSING_ANGLE
#define OPENING_SPINE_RADIUS 77 // OPENING_SPINE_RADIUS, THE DISTANCE FROM THE CENTER OF PALM TO THE FURTHEST POINT OF THE SPINES
#define OPENING_SPINE_DEPTH 4 // OPENING_SPINE_DEPTH
#define CLOSING_HEIGHT 27 // CLOSING_HEIGHT, VERTICAL DISTANCE BETWEEN THE TIP OF THE SPINE AND THE BOTTOM OF THE PALM WHEN CLOSED
#define MARGIN_OF_TOP_SOLID_DIAMETER 4 // MARGIN_OF_TOP_SOLID_DIAMETER
#define INSIDE_MARGIN_OF_BOTTOM_VOID_DIAMETER 2 // inside_margin_of_bottom_void_diameter
#endif

#if SCARE
#define PALM_DIAMETER 71 // PALM_DIAMETER
#define PALM_DIAMETER_OF_FINGER_JOINTS 92 // 
#define FINGER_LENGTH 40 // FINGER_LENGTH
#define SPINE_LENGTH 41 // SPINE_LENGTH
#define SPINE_DEPTH 5 // SPINE_DEPTH
#define OPENING_ANGLE 85 // OPENING_ANGLE
#define CLOSING_ANGLE 10 // CLOSING_ANGLE
#define OPENING_SPINE_RADIUS 136 // OPENING_SPINE_RADIUS, THE DISTANCE FROM THE CENTER OF PALM TO THE FURTHEST POINT OF THE SPINES
#define OPENING_SPINE_DEPTH 5 // OPENING_SPINE_DEPTH
#define CLOSING_HEIGHT 90 // CLOSING_HEIGHT, VERTICAL DISTANCE BETWEEN THE TIP OF THE SPINE AND THE BOTTOM OF THE PALM WHEN CLOSED
#define MARGIN_OF_TOP_SOLID_DIAMETER 4 // MARGIN_OF_TOP_SOLID_DIAMETER
#define INSIDE_MARGIN_OF_BOTTOM_VOID_DIAMETER 2 // inside_margin_of_bottom_void_diameter
#endif

#if HUBROBO
#define PALM_DIAMETER 32 // PALM_DIAMETER
#define PALM_DIAMETER_OF_FINGER_JOINTS 28 // 
#define FINGER_LENGTH 15 // FINGER_LENGTH
#define SPINE_LENGTH 15 // SPINE_LENGTH
#define SPINE_DEPTH 5 // SPINE_DEPTH
#define OPENING_ANGLE 75 // OPENING_ANGLE
#define CLOSING_ANGLE 30 // CLOSING_ANGLE
#define OPENING_SPINE_RADIUS 37 // OPENING_SPINE_RADIUS, THE DISTANCE FROM THE CENTER OF PALM TO THE FURTHEST POINT OF THE SPINES
#define OPENING_SPINE_DEPTH 5 // OPENING_SPINE_DEPTH
#define CLOSING_HEIGHT 16 // CLOSING_HEIGHT, VERTICAL DISTANCE BETWEEN THE TIP OF THE SPINE AND THE BOTTOM OF THE PALM WHEN CLOSED
#define MARGIN_OF_TOP_SOLID_DIAMETER 4 // MARGIN_OF_TOP_SOLID_DIAMETER
#define INSIDE_MARGIN_OF_BOTTOM_VOID_DIAMETER 2 // inside_margin_of_bottom_void_diameter
#endif

//MACRO
#define MY_PRINT(x) std::cout << #x"=" << x << std::endl
#define TO_FLOAT(x) static_cast<float>(x)
#define GETMAX(x) *max_element(x.begin(),x.end());
#define GETMIN(x) *min_element(x.begin(),x.end());

  // Calculate the ratio of voxel size and 1mm in order to keep the gripper size in real world regardless of voxel size
constexpr float ratio=1/(VOXEL_SIZE * 1000);
  // Reduce or magnify the gripper's parameters to fit voxel's dimension
  // Change demensions from [mm] to [voxels]
constexpr float  palm_diameter= (std::round(PALM_DIAMETER * ratio));
constexpr float  palm_diameter_of_finger_joints=(std::round(PALM_DIAMETER_OF_FINGER_JOINTS * ratio));
constexpr float  finger_length =(std::round(FINGER_LENGTH * ratio));
constexpr float  spine_length =(std::round(SPINE_LENGTH * ratio));
constexpr float  spine_depth =(std::round(SPINE_DEPTH * ratio));
constexpr float  opening_spine_radius =(std::round(OPENING_SPINE_RADIUS * ratio));

constexpr float  closing_height =(std::round(CLOSING_HEIGHT * ratio));
constexpr float  margin_of_top_solid_diameter =(std::round(MARGIN_OF_TOP_SOLID_DIAMETER * ratio));
constexpr float  inside_margin_of_bottom_void_diameter =(std::round(INSIDE_MARGIN_OF_BOTTOM_VOID_DIAMETER * ratio));


  // Set the gripper-mask size
constexpr float  gripper_mask_half_size =((static_cast<float>(palm_diameter_of_finger_joints) / 2) + finger_length + spine_length);
constexpr int  gripper_mask_size =((static_cast<float>(2 * gripper_mask_half_size)) + 1);
constexpr int  gripper_mask_height =static_cast<int>(closing_height);

  // Calculate the parameters to determine solid area and void area
constexpr float  gripper_mask_top_solid_radius =std::round((palm_diameter + margin_of_top_solid_diameter) / 2);
constexpr float  gripper_mask_clearance =(std::round((TO_FLOAT(gripper_mask_size) - palm_diameter) / 2 * std::tan((90 - OPENING_ANGLE)*(M_PI/180.0))));
constexpr float  gripper_mask_bottom_void_radius =(std::round((palm_diameter / 2 )+ (TO_FLOAT(gripper_mask_height) * std::tan((CLOSING_ANGLE)*(M_PI/180.0))) - inside_margin_of_bottom_void_diameter));


