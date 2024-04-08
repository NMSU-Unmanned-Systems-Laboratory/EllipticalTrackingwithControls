clear all
close all


%% Read in the data



% new filtered data
XYZData = {};
XYZData = [XYZData readmatrix("Data/second_data.csv")];
XYZData = [XYZData readmatrix("Data/Circle_test_2.csv")];
XYZData  =[XYZData readmatrix("Data/center_test.csv")];


% remove the beginning, etc.
XYZData{3} = XYZData{3}(10:end,:);


%% compute the error

XYZ_camera = {};
XYZ_mocap  = {};
Timestamps = {};

error_avg_old     = {};
abs_error_avg_old = {};

avg_samplingRate = {};

for i = 1:length(XYZData)
    XYZ_camera = [XYZ_camera XYZData{i}(:,2:4)];
    XYZ_mocap = [XYZ_mocap XYZData{i}(:,6:8)];
    Timestamps = [Timestamps XYZData{i}(:,1)];
    
    error_avg_old = [error_avg_old mean((XYZ_camera{i} - XYZ_mocap{i}))];
    abs_error_avg_old = [abs_error_avg_old  mean(abs(XYZ_camera{i} - XYZ_mocap{i}))]
    
    dt = mean(diff(Timestamps{i}));
    avg_samplingRate = [avg_samplingRate 1 / dt] % Hz
    
end




%% new error with a linear shift afterwards

new_camera_xyz = {};
new_error      = {};
error_avg      = {};
abs_error_avg  = {};

for i = 1:length(XYZ_camera)
new_camera_xyz = [new_camera_xyz matrix_regress(XYZ_camera{i}(:,1)',XYZ_camera{i}(:,2)',XYZ_camera{i}(:,3)',XYZ_mocap{i}(:,1)',XYZ_mocap{i}(:,2)',XYZ_mocap{i}(:,3)')'];
new_error = [new_error abs(new_camera_xyz{i} - XYZ_mocap{i})];

error_avg = [error_avg mean((new_camera_xyz{i} - XYZ_mocap{i}))];
abs_error_avg = [abs_error_avg mean(abs(new_camera_xyz{i} - XYZ_mocap{i}))]

end

%% plot the data
figure()

 for i=1:3
    subplot(2,3,i)
    hold on
    plot3(new_camera_xyz{i}(:,1),new_camera_xyz{i}(:,2),new_camera_xyz{i}(:,3),'-^','MarkerIndices',500,'color',[.2,.2,.8])
    %plot3(XYZ_camera(:,1), XYZ_camera(:,2), XYZ_camera(:,3),'-^','MarkerIndices',500,'color',[.2,.2,.8])
    plot3(XYZ_mocap{i}(:,1),XYZ_mocap{i}(:,2),XYZ_mocap{i}(:,3),'-^','MarkerIndices',500,'color',[.8 .2 .2])
    hold off
    zlim([.75 1.25])
    xlim([-1.2 1.2])
    ylim([-1 1])
    legend('Ellipse-Detection','Mo-Cap')  
end


%% Plot the error

for i=1:3

subplot(2,3,i+3)    
time = linspace(0,Timestamps{i}(end)-Timestamps{i}(1),length(new_error{i}));

plot(time, new_error{i})
ylabel('Absolute Error (m)')
xlabel('Time (Seconds)')
ylim([0 .15])
legend('x','y','z')
%title('Three-Setpoints Test Error')

end

