syms x1_sol y1_sol theta1_sol cam1_sol x2_sol y2_sol theta2_sol cam2_sol;

obj_x_sol = (-y1_sol + y2_sol + x1_sol*tan(cam1_sol + theta1_sol) - x2_sol*tan(cam2_sol + theta2_sol))/...
     (tan(cam1_sol + theta1_sol) - tan(cam2_sol + theta2_sol));
obj_y_sol = (-y1_sol*tan(cam2_sol + theta2_sol) + tan(cam1_sol + theta1_sol)*(y2_sol + (x1_sol - x2_sol)*tan(cam2_sol + theta2_sol)))/(tan(cam1_sol + theta1_sol) - tan(cam2_sol + theta2_sol));

varlist = [x1_sol y1_sol theta1_sol cam1_sol x2_sol y2_sol theta2_sol cam2_sol];

