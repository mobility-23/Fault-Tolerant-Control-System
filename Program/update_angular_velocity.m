function angular_v=update_angular_velocity(log,time_step,i)

if i<2
    angular_v = 0;
else
    angular_v = (log.veh_pose(i,3)-log.veh_pose(i-1,3))/time_step ;
end

