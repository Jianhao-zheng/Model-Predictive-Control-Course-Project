function drawOneState(X,U,Var,Deliverable)

figure_res = figure;Ts = 1/5;
T = 0:Ts:Ts*(size(X,2)-1);
switch Var
    case {"x"}
        set(figure_res,'position',[0 0 1000 500]);
        subplot(5,1,1);
        h1 = plot(T,X(1,:),'color','r',"LineWidth",2);
        ylabel("vel\_pitch (rad/s)");
        grid on;
        subplot(5,1,2);
        h2 = plot(T,X(2,:),'color','g',"LineWidth",2);
        ylabel("pitch (rad)");
        grid on;
        subplot(5,1,3);
        h3 = plot(T,X(3,:),'color','b',"LineWidth",2);
        ylabel("vel\_x (m/s)");
        grid on;
        subplot(5,1,4);
        h4 = plot(T,X(4,:),'color','k',"LineWidth",2);
        ylabel("x (m)");
        grid on;
        subplot(5,1,5);
        h5 = plot(T(1:end-1),U,'color','m',"LineWidth",2);
        ylabel("M\_alpha (N \cdot m)");
        xlabel('Time (s)');
        grid on;
    case {"y"}
        set(figure_res,'position',[0 0 1000 500]);
        subplot(5,1,1);
        h1 = plot(T,X(1,:),'color','r',"LineWidth",2);
        ylabel("vel\_roll (rad/s)");
        grid on;
        subplot(5,1,2);
        h2 = plot(T,X(2,:),'color','g',"LineWidth",2);
        ylabel("roll (rad)");
        grid on;
        subplot(5,1,3);
        h3 = plot(T,X(3,:),'color','b',"LineWidth",2);
        ylabel("vel\_y (m/s)");
        grid on;
        subplot(5,1,4);
        h4 = plot(T,X(4,:),'color','k',"LineWidth",2);
        ylabel("y (m)");
        grid on;
        subplot(5,1,5);
        h5 = plot(T(1:end-1),U,'color','m',"LineWidth",2);
        ylabel("M\_beta (N \cdot m)");
        xlabel('Time (s)');
        grid on;
    case {"z"}
        set(figure_res,'position',[0 0 500 300]);
        subplot(3,1,1);
        h1 = plot(T,X(1,:),'color','r',"LineWidth",2);
        ylabel("vel\_z (m/s)");
        grid on;
        subplot(3,1,2);
        h2 = plot(T,X(2,:),'color','g',"LineWidth",2);
        ylabel("z (m)");
        grid on;
        subplot(3,1,3);
        h3 = plot(T(1:end-1),U,'color','b',"LineWidth",2);
        ylabel("F (N)");
        xlabel('Time (s)');
        grid on;
    case {"yaw"}
        set(figure_res,'position',[0 0 500 300]);
        subplot(3,1,1);
        h1 = plot(T,X(1,:),'color','r',"LineWidth",2);
        ylabel("vel\_yaw (rad/s)");
        grid on;
        % use rad
        %subplot(3,1,2);
        %h2 = plot(T,X(2,:),'color','g',"LineWidth",2);
        %ylabel("yaw (rad)");
        % use degree
        subplot(3,1,2);
        yawDegree = X(2,:) .* (180/pi);
        h2 = plot(T,yawDegree,'color','g',"LineWidth",2);
        ylabel("yaw ({}^{\circ})");
        set(gca,'ytick',(round(min(yawDegree)):15:round(max(yawDegree))));
        ylim([round(min(yawDegree)) round(max(yawDegree))]);
        grid on;
        subplot(3,1,3);
        h3 = plot(T(1:end-1),U,'color','b',"LineWidth",2);
        ylabel("M\_gamma (N \cdot m)");
        xlabel('Time (s)');
        grid on;
end
tightfig;
print(Deliverable+"_"+Var+"_res", '-dpng');
print(Deliverable+"_"+Var+"_res", '-dpdf', '-bestfit'); % '-bestfit' | '-fillpage'

end