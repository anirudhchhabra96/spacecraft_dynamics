%matlab hexapod test script

ip = '192.168.20.3'; % Use "devicesTcpIp = Controller.EnumerateTCPIPDevices('')" to get all PI controller available on the network
port = 50000;          % Is 50000 for almost all PI controllers
axis = 'Y';
addpath (getenv ('PI_MATLAB_DRIVER'));


% Load PI_GCS_Controller if not already loaded
if(~exist('Controller','var'))
    Controller = PI_GCS_Controller();
end
if(~isa(Controller,'PI_GCS_Controller'))
    Controller = PI_GCS_Controller();
end
%% Start connection
% (if not already connected)
try
    boolPIdeviceConnected = false;
    if ( exist ( 'PIdevice', 'var' ) ), if ( PIdevice.IsConnected ), boolPIdeviceConnected = true; end; end
    if ( ~(boolPIdeviceConnected ) )
        PIdevice = Controller.ConnectTCPIP ( ip, port ) ;
    end

    % query controller identification string
    connectedControllerName = PIdevice.qIDN();

    % initialize PIdevice object for use in MATLAB
    PIdevice = PIdevice.InitializeController ();

    %% Startup stage
    % switch servo on for axis
    switchOn    = 1;
    PIdevice.SVO ( axis, switchOn );

    %% Get min/max position limits
    % determine the allowed travel range of the stage
    minimumPosition = PIdevice.qTMN ( axis );
    maximumPosition = PIdevice.qTMX ( axis );
    travelRange = ( maximumPosition - minimumPosition );

    %% Do absolute motions

    % PIdevice.MOV (axis, 0);
    % while(0 ~= PIdevice.IsMoving () )
    %     pause ( 0.1 );
    %     fprintf('.');
    % end
    % PIdevice.MOV (axis, 20);
    % while(0 ~= PIdevice.IsMoving () )
    %     pause ( 0.1 );
    %     fprintf('.');
    % end
    % PIdevice.MOV (axis, 0);
    % while(0 ~= PIdevice.IsMoving () )
    %     pause ( 0.1 );
    %     fprintf('.');
    % end
    % PIdevice.MOV (axis, -20);
    % while(0 ~= PIdevice.IsMoving () )
    %     pause ( 0.1 );
    %     fprintf('.');
    % end
    % PIdevice.MOV (axis, 0);

    % PIdevice.MOV (axis, 0);
    % while(0 ~= PIdevice.IsMoving () )
    %     pause ( 0.1 );
    %     fprintf('.');
    % end
    %
    % PIdevice.MOV (axis, 20);
    % while(0 ~= PIdevice.IsMoving () )
    %     pause ( 0.1 );
    %     fprintf('.');
    % end
    %
    % PIdevice.MOV (axis, 0);
    %
    % PIdevice.MOV (axis, -20);
    %
    % PIdevice.MOV (axis, 0);
    %%
    % PIdevice.MOV (axis, 0);
    % pause(1);
    % PIdevice.MOV (axis, 20);
    %

    PIdevice.VLS(20);
    PIdevice.MOV ('X Y Z U V W', [30, 0, 0, 0, 0, 0]);
    while(0 ~= PIdevice.IsMoving () )
        pause ( 0.1 );
    end


    userinput=input("Press enter to continue...\n");

    % Trajectory Source, 1 = Motion path determined by consecurive MOV
    % Trajectory Execuition, 1 = Motion Profile is stored in a buffer
    % before execution
   
    % SPA {ItemID, PamID, PamValue}
    PIdevice.SPA(axis, 0x19001900, 1);
    PIdevice.SPA(axis, 0x19001901, 1);
    % PIdevice.DRC(2,'1',72);

    % Set Cycle time in ms
    PIdevice.SCT(300);

    deg_per_point = 4;
    x = [];
    for i = 0:deg_per_point*pi/180:2*3.14
        x = [x,cos(i)];
    end

    y = [];
    for i = 0:deg_per_point*pi/180:2*3.14
        y = [y, sin(i)];
    end

    u = [0, 1, 0, -1, 0];
    v = [1, 0, -1, 0, 1];

    u = -10*u;
    v = 10*v;

    t = [0, pi/2, pi, 3*pi/2, 2*pi];
    tq = 0:deg_per_point*pi/180:2*3.14;
    ut = interp1(t, u, tq);
    vt = interp1(t, v, tq);

    x = 30*x;
    y = 30*y;
    for i = 1:length(x)
        PIdevice.MOV('X Y U V', [x(i), y(i), ut(i), vt(i)]);
    end


    % clc;
    % out = PIdevice.qDRR(2,1,2);
    % disp(out);


    true_x = [];
    true_y = [];
    %% switch servo off
    while(0 ~= PIdevice.IsMoving () )
        pause ( 0.1 );
        true_xy = PIdevice.qPOS('X Y')
        true_x = [true_x, true_xy(1)];
        true_y = [true_y, true_xy(2)];
        fprintf('.');
    end
    fprintf('\n');

    plot(true_x, true_y, 'o');


    PIdevice.MOV ('X Y Z U V W', [30, 0, 0, 0, 0, 0]);
    while(0 ~= PIdevice.IsMoving () )
        pause ( 0.1 );
    end


    % switchOff    = 0;
    % PIdevice.SVO ( axis, switchOff );

    %% If you want to close the connection
    PIdevice.VLS(2);
    PIdevice.CloseConnection ();

    %% If you want to unload the dll and destroy the class object
    Controller.Destroy ();
    clear Controller;
    clear PIdevice;
catch
    Controller.Destroy;
    clear Controller;
    clear PIdevice;
    rethrow(lasterror);
end

