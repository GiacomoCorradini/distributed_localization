function [xA,yA,x1A,y1A] = object_detection(x1,y1,theta1,x2,y2,phi2,camera1,camera2)

    % SUMMURY
    % 

    theta2 = theta1 + phi2;

    A = [-tan(theta1+camera1) 1;
         -tan(theta2+camera2) 1];
    
    B = [-tan(theta1+camera1).*x1 + y1;
         -tan(theta2+camera2).*x2 + y2];
    
    [pointA] = linsolve(A,B);

    xA = pointA(1);
    yA = pointA(2);

    x1A = xA - x1;
    y1A = yA - y1;

end