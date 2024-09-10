  // ****** General Parameters ******* //
#include <cmath>

//choose a model WARNING: only put one to true or you wiill have conflict
#define LIMBERO true
#define SCARE false
#define HUBROBO false

  // MatchingSettings
#define VOXEL_SIZE 0.001 // voxel size [m]
#define THRESHOLD 90// threshold of numbers of solid voxels within the subset (TSV) (SCAR-E: 120)
#define DELETE_LOWER_TARGETS true // delete the targets whose z positions are lower than a limit value: on or off
#define GRASPABILITY_THRESHOLD 90 // Graspability threshold. Above which we can call it graspable with great certainty
#define FINNER_INTERPOLATION false //If you need to add more points at interpolation process because the received cloud lack density

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



  // Set the gripper-mask size
constexpr float  GRIPPER_MASK_HALF_SIZE =((PALM_DIAMETER_OF_FINGER_JOINTS / 2) + FINGER_LENGTH + SPINE_LENGTH);
constexpr float  GRIPPER_MASK_SIZE =(2 * GRIPPER_MASK_HALF_SIZE);
constexpr float  GRIPPER_MASK_HEIGHT =(CLOSING_HEIGHT);

  // Calculate the parameters to determine solid area and void area
constexpr float  GRIPPER_MASK_TOP_SOLID_RADIUS =(PALM_DIAMETER + MARGIN_OF_TOP_SOLID_DIAMETER) / 2;
constexpr float  GRIPPER_MASK_CLEARANCE =(((GRIPPER_MASK_SIZE) - PALM_DIAMETER) / 2 * std::tan((90 - OPENING_ANGLE)*(M_PI/180.0)));
constexpr float  GRIPPER_MASK_BOTTOM_VOID_RADIUS =((PALM_DIAMETER / 2)+ ((GRIPPER_MASK_HEIGHT) * std::tan((CLOSING_ANGLE)*(M_PI/180.0))) - INSIDE_MARGIN_OF_BOTTOM_VOID_DIAMETER);

    /*
    if(((point.z>max_z-GRIPPER_MASK_CLEARANCE)&&(point.z<max_z) &&(distance_from_center_of_layer < grippable_radius) && (distance_from_center_of_layer > unreachble_radius))||
        ((point.z <max_z-GRIPPER_MASK_CLEARANCE) && (distance_from_center_of_layer > GRIPPER_MASK_BOTTOM_VOID_RADIUS))||
        (((z_subscript) < std::round(GRIPPER_MASK_CLEARANCE*1.75)) && (TO_FLOAT(z_subscript) > GRIPPER_MASK_CLEARANCE))) //added conditio to ave a bigger hole at the bottom of gripper)
    */
    //(point.z >max_z-GRIPPER_MASK_HEIGHT) && 


