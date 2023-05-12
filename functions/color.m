%% Define new colors
function col = color(name)
switch name
    case {'green' , 1}
        col = [0.3660 0.5740 0.1880];
    case {'orange', 2}
        col = [0.8500 0.3250 0.0980];  
    case {'blue' , 3}
        col = [0 0.1470 1];
    case {'orchid', 4}
        col = [1 0 1];
    case {'yellow', 5}
        col = [1 1 0];


    case {'cobalt_green', 11}
        col = [0.3660 0.9740 0.5880];
    case {'light_orange', 12}
        col = [0.9290 0.5940 0.1250]; 
    case {'ligth_blue', 13}
        col = [0.3010 0.7450 0.9330];
    case {'purple', 14}
        col = [0.4940 0.1840 0.5560];
    case {'gold', 15}
        col = [0.9290 0.840 0.1250];

    otherwise
        warning('Color name not recognized. Color set to black.')
        col = [0	0	0]./ 255;
end
end