function Robotics_FinalProject_Team20()

    clear all
    clc

    disp('Select the Arm from the following list : ');

    disp('1 - 3R Arm');
    disp('2 - SCARA (RRPR) Arm'); 
    disp('3 - Stanford Manipulator');
    disp('4 - RRPRP Arm');
    
    str = input('Please choose the Arm to be shown (for example for Stanford Manipulator Enter 4)  -   ');

    switch str
        case 1
            RRR()
        case 2
            SCARA_RRPR()
        case 3
            Stanford_Manipulator()
        case 4
            RRPRP()

    end

    function Stanford_Manipulator()
        
        a = sym('a');
        alpha = sym('alpha');
        d = sym('d');
        theta = sym('theta');
        
        revolute_joint  = [1 1 0 1 1 1]; % Modify the number of revolute(1) and prisimatic(0) joints
        prismatic_joint = (-1)*(revolute_joint-1);
        N   = 6;                % Number of Joints                   
        q   = sym('q',[N,1]);   % joints symbolic variables
        d1 = sym('d1');
        d2 = sym('d2');
        d6 = sym('d6');
        
        % User defined DH Table
        disp('DH Table  -----');
        DHTABLE = [     -pi/2   0       d1      q(1);
                        pi/2    0       d2      q(2);
                        0       0       q(3)    -pi;
                        -pi/2   0       0       q(4);
                        pi/2    0       0       q(5);
                        0       0       d6      q(6)    ]

        DH = [  cos(theta)  -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta);
                  0             sin(alpha)             cos(alpha)            d;
                  0               0                      0                   1  ];        
        
        %% Direct Kinematics
        A = cell(1,N);
        for i = 1:N
            alpha   = DHTABLE(i,1);
            a       = DHTABLE(i,2);
            d       = DHTABLE(i,3);
            theta   = DHTABLE(i,4);
            A{i} = subs(DH);
        end
        clear alpha a d theta

        % Building base-to-end-effector transformation matrix 
        T = eye(4);
        for i = 1:N 
            T =  T * A{i};
            T = simplify(T);
        end
        
        P05 = T(1:3, 4);   % poition vector from transformation matrix
        R05 = T(1:3, 1:3); % rotation matrix from transformation matrix
        disp('Transformation Matrix (Direct Kinematics)  -----');
        T        
        
        %% Calculating Geometric Jacobian matrix        
        % linear component
        Jl = zeros(0,0);
        for i = 1:N
           Jl = simplify( [Jl , diff(P05,q(i))] );
        end
        
        % angular component
        Ja = zeros(0,0);
        R = eye(3);
        z = [0 0 1]';
        for i = 1:N
            Ja  = [Ja , R * z * revolute_joint(i)];
            R   = simplify( R * (A{i}(1:3,1:3)) );
        end
        clear z R
        disp('Geometric Jacobian  -----');
        J = [Jl ; Ja]
        
        % joint values to plot (MODIFY HERE!)
        k = 0.5;
        joint_value = [0, 0, k*1.5, pi/2, -pi/2, 0];
        
        k = 0.5;
        
        % dh table without non constant parameters
        DH = DHTABLE;
        DH = subs(DHTABLE, {d1,d2,d6}, {k,k,k});
        for i = 1:N
            DH = subs(DH, q(i), joint_value(i));
        end
        DH = sym2double(DH);
        
        % Robot Plot
        body1 = rigidBody('body1');
        body2 = rigidBody('body2');
        body3 = rigidBody('body3');
        body4 = rigidBody('body4');
        body5 = rigidBody('body5');
        body6 = rigidBody('body6');
        body = [body1, body2, body3, body4, body5, body6];
        
        jnt1 = rigidBodyJoint('jnt1','revolute');
        jnt2 = rigidBodyJoint('jnt2','revolute');
        jnt3 = rigidBodyJoint('jnt3','prismatic');
        jnt4 = rigidBodyJoint('jnt4','revolute');
        jnt5 = rigidBodyJoint('jnt5','revolute');
        jnt6 = rigidBodyJoint('jnt6','revolute');
        jnt = [jnt1, jnt2, jnt3, jnt4, jnt5, jnt6];
        
        robot = robot_conf_build(body, jnt, DH);
        joint_conf = joint_conf_build(robot, joint_value);
        disp('Robot Details for PLOT  -----')
        showdetails(robot)
        show(robot, joint_conf);
        axis on
    end

    function RRPRP()
        clear all
        clc

        a = sym('a');
        alpha = sym('alpha');
        d = sym('d');
        theta = sym('theta');

        revolute_joint  = [1 1 0 1 0]; % Modify the number of revolute(1) and prisimatic(0) joints
        prismatic_joint = (-1)*(revolute_joint-1);

        %MODIFY HERE
        N   = 5;                 % joint number                   
        q   = sym('q',[N,1]);    % joints symbolic variables
        d1  = sym('d1');         % z offset along link 1

        % User defined DH Table
        disp('DH Table  -----');
        DHTABLE = [     pi/2    0   d1      q(1);
                        pi/2    0   0       q(2);
                        pi/2    0   q(3)    pi;
                        pi/2    0   0       q(4);
                        0       0   q(5)    0     ]

        %% General Denavit-Hartenberg trasformation matrix
        DH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                0             sin(alpha)             cos(alpha)            d;
                0               0                      0                   1  ];

        %% Direct Kinematics
        A = cell(1,N);
        for i = 1:N
            alpha   = DHTABLE(i,1);
            a       = DHTABLE(i,2);
            d       = DHTABLE(i,3);
            theta   = DHTABLE(i,4);
            A{i} = subs(DH);
        end
        clear alpha a d theta
        % Build base-to-end-effector transformation matrix 
        T = eye(4);
        for i = 1:N 
            T =  T * A{i};
            T = simplify(T);
        end

        P05 = T(1:3, 4);   % poition vector
        R05 = T(1:3, 1:3); % rotation matrix
        disp('Transformation Matrix (Direct Kinematics)  -----');
        T 

        %% Calculating Geometric Jacobian matrix
        % linear component
        Jl = zeros(0,0);
        for i = 1:N
            Jl = simplify( [Jl , diff(P05,q(i))] );
        end

        % angular component
        Ja = zeros(0,0);
        R = eye(3);
        z = [0 0 1]';
        for i = 1:N
            Ja  = [Ja , R * z * revolute_joint(i)];
            R   = simplify( R * (A{i}(1:3,1:3)) );
        end
        clear z R
        % full geometric Jacobian
        disp('Geometric Jacobian  -----');
        J = [Jl ; Ja]

        % joint values to plot (MODIFY HERE!)
        k = 0.5;
        joint_value = [0, pi/2, k, pi/2, k];

        k = 0.5;	% fixed magnitude for prismatic joints
        %dh table without non constant parameters
        DH = DHTABLE;
        DH = subs(DHTABLE, d1, k);
        for i = 1:N
            DH = subs(DH, q(i), joint_value(i));
        end
        DH = sym2double(DH);

        % Robot Plot
        body1 = rigidBody('body1');
        body2 = rigidBody('body2');
        body3 = rigidBody('body3');
        body4 = rigidBody('body4');
        body5 = rigidBody('body5');
        body = [body1, body2, body3, body4, body5];

        jnt1 = rigidBodyJoint('jnt1','revolute');
        jnt2 = rigidBodyJoint('jnt2','revolute');
        jnt3 = rigidBodyJoint('jnt3','prismatic');
        jnt4 = rigidBodyJoint('jnt4','revolute');
        jnt5 = rigidBodyJoint('jnt5','prismatic');
        jnt = [jnt1, jnt2, jnt3, jnt4, jnt5];

        robot = robot_conf_build(body, jnt, DH);
        joint_conf = joint_conf_build(robot, joint_value);
        disp('Robot Details for PLOT  -----')
        showdetails(robot)
        show(robot, joint_conf);
        axis on
    end

    function RRR()

        a = sym('a');
        alpha = sym('alpha');
        d = sym('d');
        theta = sym('theta');

        revolute_joint  = [1 1 1] %MODIFY HERE

        %MODIFY HERE
        N   = 3;                 % joint number                   
        q   = sym('q',[N,1]);    % joints symbolic variables
        d1 = sym('d1');
        d2 = sym('d2');
        a3 = sym('a3');

        % User defined DH Table
        disp('DH Table  -----');
        DHTABLE = [     pi/2    0       d1      q(1);
                        pi/2    0       0      q(2);
                        0       a3      d2       q(3);    ]

        %% General Denavit-Hartenberg trasformation matrix
        DH = [  cos(theta)  -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta);
                0             sin(alpha)             cos(alpha)            d;
                0               0                      0                   1  ];


        %% Direct Kinematics
        A = cell(1,N);
        for i = 1:N
            alpha   = DHTABLE(i,1);
            a       = DHTABLE(i,2);
            d       = DHTABLE(i,3);
            theta   = DHTABLE(i,4);
            A{i} = subs(DH);
        end
        clear alpha a d theta
        % Building base-to-end-effector transformation matrix 
        T = eye(4);
        for i = 1:N 
            T =  T * A{i};
            T = simplify(T);
        end

        P05 = T(1:3, 4);   % poition vector
        R05 = T(1:3, 1:3); % rotation matrix
        disp('Transformation Matrix (Direct Kinematics)  -----');
        T 

        %% Calculating Geometric Jacobian matrix
        % linear component
        Jl = zeros(0,0);
        for i = 1:N
        Jl = simplify( [Jl , diff(P05,q(i))] );
        end

        % angular component
        Ja = zeros(0,0);
        R = eye(3);
        z = [0 0 1]';
        for i = 1:N
            Ja  = [Ja , R * z * revolute_joint(i)];
            R   = simplify( R * (A{i}(1:3,1:3)) );
        end
        clear z R
        % full geometric Jacobian
        disp('Geometric Jacobian  -----');
        J = [Jl ; Ja]

        % joint values to plot (MODIFY HERE!)
        k = 0.5;
        joint_value = [-pi/2, -2*pi/5, pi];

        k = 0.5;	% fixed magnitude for prismatic joints
        %dh table without non constant parameters
        DH = DHTABLE;
        DH = subs(DHTABLE, {d1,d2,a3}, {k,k,k});    %modify here
        for i = 1:N
            DH = subs(DH, q(i), joint_value(i));
        end
        DH = sym2double(DH)

        % Robot Plot
        body1 = rigidBody('body1');
        body2 = rigidBody('body2');
        body3 = rigidBody('body3');
        body = [body1, body2, body3];

        jnt1 = rigidBodyJoint('jnt1','revolute');
        jnt2 = rigidBodyJoint('jnt2','revolute');
        jnt3 = rigidBodyJoint('jnt3','revolute');
        jnt = [jnt1, jnt2, jnt3];

        robot = robot_conf_build(body, jnt, DH);
        joint_conf = joint_conf_build(robot, joint_value);
        disp('Robot Details for PLOT  -----');
        showdetails(robot)
        show(robot, joint_conf);
        axis on
    end

    function SCARA_RRPR()

        clear all
        clc

        a = sym('a');
        alpha = sym('alpha');
        d = sym('d');
        theta = sym('theta');

        revolute_joint  = [1 1 0 1];    %MODIFY HERE
        prismatic_joint = (-1)*(revolute_joint-1);
        
        %MODIFY HERE
        N=4;
        q   = sym('q',[N,1]);    % joints symbolic variables
        d3 = sym('d1');
        a1 = sym('a1');
        a2 = sym('a2');

        % User defined DH Table
        disp('DH Table  -----');
        DHTABLE = [  0   0   0   q(1);
                    0   a1  0   q(2);
                    pi  a2  d3   0;
                    0   0   0   q(4)]
        
        %% General Denavit-Hartenberg trasformation matrix
        DH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                0             sin(alpha)             cos(alpha)            d;
                0               0                      0                   1];

        %% Direct Kinematics
        A = cell(1,N);
        for i = 1:N
            alpha = DHTABLE(i,1);
            a = DHTABLE(i,2);
            d = DHTABLE(i,3);
            theta = DHTABLE(i,4);
            A{i} = subs(DH);
        end
        clear alpha a d theta
        % Building base-to-end-effector transformation matrix
        T = eye(4);
        for i = 1:N 
            T =  T * A{i};
            T = simplify(T);
        end

        T = eye(4);

        for i=1:N 
            T = T*A{i};
            T = simplify(T);
        end

        P05 = T(1:3, 4);   % poition vector
        R05 = T(1:3, 1:3); % rotation matrix
        disp('Transformation Matrix (Direct Kinematics)  -----');
        T

        %% Calculating Geometric Jacobian matrix
        % linear component
        Jl = zeros(0,0);
        for i = 1:N
        Jl = simplify( [Jl , diff(P05,q(i))] );
        end

        % angular component
        Ja = zeros(0,0);
        R = eye(3);
        z = [0 0 1]';
        for i = 1:N
            Ja  = [Ja , R * z * revolute_joint(i)];
            R   = simplify( R * (A{i}(1:3,1:3)) );
        end
        clear z R
        % full geometric Jacobian
        disp('Geometric Jacobian  -----');
        J = [Jl ; Ja]

        % joint values to plot (MODIFY HERE!)
        k = 0.5;
        joint_value = [0, pi/2, k, pi/2];

        k = 0.5;     % fixed magnitude for prismatic joints
        %dh table without non constant parameters
        DH = DHTABLE;
        DH = subs(DHTABLE, {a1,a2,d3}, {k,k,k});
        for i = 1:N
            DH = subs(DH, q(i), joint_value(i));
        end
        DH = sym2double(DH);

        % Robot Plot
        body1 = rigidBody('body1');
        body2 = rigidBody('body2');
        body3 = rigidBody('body3');
        body4 = rigidBody('body4');
        body = [body1, body2, body3, body4];

        jnt1 = rigidBodyJoint('jnt1','revolute');
        jnt2 = rigidBodyJoint('jnt2','revolute');
        jnt3 = rigidBodyJoint('jnt3','prismatic');
        jnt4 = rigidBodyJoint('jnt4','revolute');
        jnt = [jnt1, jnt2, jnt3, jnt4];

        robot = robot_conf_build(body, jnt, DH);
        joint_conf = joint_conf_build(robot, joint_value);
        disp('Robot Details for PLOT  -----');
        showdetails(robot)
        show(robot, joint_conf);
        axis on
    end

    % Function to generate Robot configuration
    function robot = robot_conf_build(body, jnt, DH)
        robot = rigidBodyTree;
        N = size(body,2);
        
        % [alpha a d theta] --> [a  alpha  d  theta]
        DH = [DH(:,2), DH(:,1), DH(:,3), DH(:,4)];
         
        for i = 1:N
            %transformation from jnt(i-1) to jnt(i)
            setFixedTransform(jnt(i), DH(i,:), 'dh');
            body(i).Joint = jnt(i);
            % attach bodies
            if i==1
                addBody(robot, body(i), 'base');% attach body 1 to the base frame
            else
                addBody(robot, body(i), body(i-1).Name);
            end
        end
    end
    
    % Function to generate Joint Configuration
    function joint_conf = joint_conf_build(robot, joint_value)
        N = size(robot.Bodies,2);
        % build joint configuration object
        joint_conf = homeConfiguration(robot);
        for i = 1:N
            joint_conf(i).JointPosition = joint_value(i);
        end
    end

    % Funtion to transform symbolic variables to double
    function ret = sym2double(m)
        ret = zeros(size(m,1), size(m,2));
        for i = 1:size(m,1)
            for j = 1:size(m,2)
                ret(i,j) = double(m(i,j));
            end
        end
    end

end