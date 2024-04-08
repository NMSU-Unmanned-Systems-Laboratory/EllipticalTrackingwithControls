function [data4,b1] = matrix_regress(x,y,z,x2,y2,z2)


size=length(x); % number of points in the data, you can specify any number above 1
%disp(size)
error=0.10; % assuming there is 5% error between the two data sets, you can specify any number below 1

c=ones(1,size); % all one vector

data1modified=[x;y;z;c]; % modify data1 by including one row for shift (bias)

%plot3(data1(1,:),data1(2,:),data1(3,:)); % plot data1


data3 = [x2;y2;z2];
%plot3(data3(1,:),data3(2,:),data3(3,:)); % plot data3

% now we only know data 1 and data 3, our job is to find the transfer matrix b0

% this is the magic equation from PSU website, we use only data1 and data3 to obtain the transfer matrix
b1=transpose(inv(data1modified*transpose(data1modified))*data1modified*transpose(data3));

% we test if the calculated transfer matrix is good, we use it to generate data4 from data1
data4=b1*data1modified;

%plot3(data4(1,:),data4(2,:),data3(3,:)); % plot data4, data4 should be very closed to data3

end
