function title_name = set_title_name(path_tracking_alg)

switch (path_tracking_alg)

    case 'Nominal_MPC'
        title_name = 'Path Tracking - Nominal MPC';

    case 'Tube_MPC'
        title_name = 'Path Tracking - Tube MPC';
    
    case 'GP_MPC'
        title_name = 'Path Tracking - GP MPC';
        
    case 'Tube_GP_MPC'
        title_name = 'Path Tracking - Tube GP MPC';
        
    case 'Nominal_MPC_Disturbance'
        title_name = 'Path Tracking - Nominal MPC Disturbance';
        
    otherwise
        title_name = 'Error - No Algorithm';
        disp('There is no this path tracking algorithm!');
end

