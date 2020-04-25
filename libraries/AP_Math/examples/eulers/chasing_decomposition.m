function main
    % forward: front is +
    % lateral: right is +
    % throttle: down is +
    thrusts = [3 4 5];
    
    angles = [45 90 135 180 -45 -90 -135 -180];
    
    disp('roll 0 pitch 0 yaw changed')
    for n = 1:8
        decomped = decomp([0 0 angles(n)], thrusts);
        fprintf('yaw %f forward %f lateral %f throttle %f\n',angles(n), decomped(1), decomped(2), decomped(3));
    end
    
    disp('roll 0 yaw 0 pitch changed')
    for n = 1:8
        decomped = decomp([0 angles(n) 0], thrusts);
        fprintf('pitch %f forward %f lateral %f throttle %f\n',angles(n), decomped(1), decomped(2), decomped(3));
    end
    
    disp('pitch 0 yaw 0 roll changed')
    for n = 1:8
        decomped = decomp([angles(n) 0 0], thrusts);
        fprintf('roll %f forward %f lateral %f throttle %f\n',angles(n), decomped(1), decomped(2), decomped(3));
    end
    
    disp('roll 90 pitch 90 yaw 90')
    decomped = decomp([90 90 90], thrusts);
    fprintf('forward %f lateral %f throttle %f\n',decomped(1), decomped(2), decomped(3));
    
    disp('roll 180 pitch 180 yaw 180')
    decomped = decomp([180 180 180], thrusts);
    fprintf('forward %f lateral %f throttle %f\n',decomped(1), decomped(2), decomped(3));
    
    disp('roll 45 pitch 45 yaw 45')
    decomped = decomp([45 45 45], thrusts);
    fprintf('forward %f lateral %f throttle %f\n',decomped(1), decomped(2), decomped(3));
    
    disp('roll 45 pitch 45 yaw 99')
    decomped = decomp([45 45 99], thrusts);
    fprintf('forward %f lateral %f throttle %f\n',decomped(1), decomped(2), decomped(3));
    
    disp('roll 153.2109 pitch 88.8527 yaw -44.1103')
    decomped = decomp([153.2109 88.8527 -44.1103], thrusts);
    fprintf('forward %f lateral %f throttle %f\n',decomped(1), decomped(2), decomped(3));
    
    disp('roll 3.3677 pitch 88.8527 yaw 166.1753')
    decomped = decomp([3.3677 88.8527 166.1753], thrusts);
    fprintf('forward %f lateral %f throttle %f\n',decomped(1), decomped(2), decomped(3));
end

function decomped = decomp(eulAngles, thrusts)
    nedEul = [0 0 0];
    curEul = eulAngles;
    
    ned = eulVector(nedEul(1), nedEul(2), nedEul(3)); % ZYX euler
    cur = eulVector(curEul(1), curEul(2), curEul(3));
    
    %plotEulInNEDAxis(nedEul,['g','g','g'],'-',1)
    %plotEulInNEDAxis(curEul,['r','r','r'],'-',1)

    nedRotB2N = eul2rotm(ned); % body -> ned
    nedRotN2B = transpose(nedRotB2N);
    zNedRotN2B = nedRotN2B * [0;0;1]; % roigin att z axis at body axis

    curRotB2N = eul2rotm(cur);
    curRotN2B = transpose(curRotB2N);
    zCurRotN2B = curRotN2B * [0;0;1]; % cur att z axis at body axis
%     disp('zCurRotN2B')
%     disp(zCurRotN2B)

    % rotat in body axis
    red = n2c(zNedRotN2B, zCurRotN2B);
%     disp('red')
%     disp(red)
    
    % q_b^e
    rotm = quat2rotm(red);
    %disp(rotm)
    
    decomped = rotm * thrusts';
    
    %plotQuatInNEDAxis(red ,['k','k','k'],'--',3)
end

function vec = eulVector(roll, pitch, yaw)
    vec = [deg2rad(yaw) deg2rad(pitch) deg2rad(roll)];
end

function Q = n2c(src, dst)
    eps=1e-5;
    cr = cross(src,dst);
    dt = dot(src,dst);
    if (norm(cr) < eps && dt < 0) 
        %disp('norm < eps')
        %cr = src.abs();
        cr = abs(src);
        %disp('abs cr')
        %disp(cr)
        if (cr(1) < cr(2)) 
            if (cr(1) < cr(3)) 
                cr = [1, 0, 0];
            else 
                cr = [0, 0, 1];
            end
        else
            if (cr(2) < cr(3)) 
                %fprintf('test %f %f\n', cr(2), cr(3))
                cr = [0, 1, 0];
            else 
                cr = [0, 0, 1];               
            end
        end    
        %disp('set cr')
        %disp(cr)
        q(1) = 0;
        cr = cross(src,cr);
        %disp('new cross cr')
        %disp(cr)
    else 
        q(1) = dt + sqrt(norm(src)^2 * norm(dst)^2);
    end
    q(2) = cr(1);
    q(3) = cr(2);
    q(4) = cr(3);
    Q=[q(1),q(2),q(3),q(4)];
    Q=quatnormalize(Q);
end

function plotEulInNEDAxis(target,color,linestyle,linewidth)
    euler = eulVector(target(1), target(2), target(3));
    b2n = eul2rotm(euler);
    n2b = transpose(b2n);
    
    center = [0 0 0];
    
    x_axis=[1+center(1),center(2),center(3)];
    y_axis=[center(1),-1+center(2),center(3)];
    z_axis=[center(1),center(2),-1+center(3)];
    
    x_axis = n2b*x_axis';
    y_axis = n2b*y_axis';
    z_axis = n2b*z_axis';
    
    O_point=[center(1),center(2),center(3)];
    
    plot3([O_point(1),x_axis(1)],[O_point(2),x_axis(2)],[O_point(3),x_axis(3)],'Color',color(1),'LineStyle',linestyle,'LineWidth',linewidth,'MarkerEdgeColor','b');
    hold on;
    plot3([O_point(1),y_axis(1)],[O_point(2),y_axis(2)],[O_point(3),y_axis(3)],'Color',color(2),'LineStyle',linestyle,'LineWidth',linewidth);
    hold on;
    plot3([O_point(1),z_axis(1)],[O_point(2),z_axis(2)],[O_point(3),z_axis(3)],'Color',color(3),'LineStyle',linestyle,'LineWidth',linewidth);
      
    hold on;
    text(x_axis(1),x_axis(2),x_axis(3),'x');
    text(y_axis(1),y_axis(2),y_axis(3),'y');
    text(z_axis(1),z_axis(2),z_axis(3),'z');
   
    xlim([-1.1,1.1]);
    ylim([-1.1,1.1]);
    zlim([-1.1,1.1]);
end

function plotQuatInNEDAxis(target,color,linestyle,linewidth)

    rotm = quat2rotm(target);

    center = [0 0 0];
    
    x_axis=[1+center(1),center(2),center(3)];
    y_axis=[center(1),-1+center(2),center(3)];
    z_axis=[center(1),center(2),-1+center(3)];
    
    x_axis = rotm*x_axis';
    y_axis = rotm*y_axis';
    z_axis = rotm*z_axis';
    
    O_point=[center(1),center(2),center(3)];
    
    plot3([O_point(1),x_axis(1)],[O_point(2),x_axis(2)],[O_point(3),x_axis(3)],'Color',color(1),'LineStyle',linestyle,'LineWidth',linewidth,'MarkerEdgeColor','b');
    hold on;
    plot3([O_point(1),y_axis(1)],[O_point(2),y_axis(2)],[O_point(3),y_axis(3)],'Color',color(2),'LineStyle',linestyle,'LineWidth',linewidth);
    hold on;
    plot3([O_point(1),z_axis(1)],[O_point(2),z_axis(2)],[O_point(3),z_axis(3)],'Color',color(3),'LineStyle',linestyle,'LineWidth',linewidth);
      
    hold on;
    text(x_axis(1),x_axis(2),x_axis(3),'x');
    text(y_axis(1),y_axis(2),y_axis(3),'y');
    text(z_axis(1),z_axis(2),z_axis(3),'z');
   
    xlim([-1.1,1.1]);
    ylim([-1.1,1.1]);
    zlim([-1.1,1.1]);
end
