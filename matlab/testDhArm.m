% Test the MATLAB wrapper version of DhArm

% Create 2 links
I_3x3 = eye(3); % default inertia matrix
joint_type = 82; % 'R' for revolute joint
point = gtsam.Point3(-1, 0, 0);
link1 = manipulator.DhLink(0, 0, 2, 0, joint_type, 1, point, I_3x3, -180, 180, 0, 10000, 0, 10000, 0, 10000, 0);
link2 = manipulator.DhLink(0, 0, 2, 0, joint_type, 1, point, I_3x3, -180, 180, 0, 10000, 0, 10000, 0, 10000, 0);

% Create a vector of them
links = manipulator.DhLinkVector;
links.push_back(link1);
links.push_back(link2);

% Create the manipulator
robot = manipulator.DhArm(links, gtsam.Pose3(), gtsam.Pose3());

% Check that the number of links is correct
gtsam.EXPECT('robot.numLinks()==2',robot.numLinks()==2);