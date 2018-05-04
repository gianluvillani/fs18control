% MAP CREATOR
% 9.10.2017
% 
% define the map with circuit CONES and starting position
% map dimension, cones values and car initial position values
% are defined in settings.
% number of cars = 1
% number of starting cones = 2
% number of BLUE and YELLOW cones = unlimited
%
%_______________________MAP SETTINGS____________________________

MAX_X=100;                           % 2D map X dimension
MAX_Y=100;                           % 2D map Y dimension

%_____________________VALUES SETTINGS___________________________

blue_value=1;
yellow_value=2;
start_value=3;
starting_position=4;

%______________________INITIALIZATION___________________________

MAP=zeros(MAX_X,MAX_Y);

x_val = 1;
y_val = 1;
j=0;
figure()
axis([1 MAX_X+1 1 MAX_Y+1])  % plot MAP
grid on;
hold on;


%_________________________INPUTS________________________________

xlabel('Select the car INITIAL POSITION ','Color','black');
but=0;
while (but ~= 1)
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=starting_position;
end
plot(xval+.5,yval+.5,'g*');


xlabel('Select STARTING CONES using the Left Mouse button,to select the last obstacle use the Right button','Color','black');
%but=1;
%while but == 1
for i=0:1
    [xval,yval,~] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=start_value;
    plot(xval+.5,yval+.5,'dk');
end;

xlabel('Select BLUE CONES using the Left Mouse button,to select the last obstacle use the Right button','Color','blue');
but=1;
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=blue_value;
    plot(xval+.5,yval+.5,'bo');
end;

xlabel('Select "YELLOW" CONES using the Left Mouse button,to select the last obstacle use the Right button','Color','magenta');
but=1;
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=yellow_value;
    plot(xval+.5,yval+.5,'mo');
end;

dlmwrite('map_circuit.txt', MAP, 'delimiter', '\t');
xlabel('DONE!','Color','green');
