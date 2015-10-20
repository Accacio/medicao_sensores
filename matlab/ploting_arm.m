function [ ] = ploting_arm( shoulder_angle, elbow_angle, Dca, Dcf_array, tensor_array, t_angle_array )

%tension plot, plot the position of the forearm and the force tension vs
%the distance of Dcf.


%Global variables
global arm_xini;
global arm_yini;
global forearm_xini;
global forearm_yini;
global La;
global Lf;
global Lh;
global Dfcm;
global Dd;

for i=1:size(elbow_angle,2)
    
    phi_shoulder=pi/2-shoulder_angle;  %shoulder angle rotated
    phi_elbow=pi/2-elbow_angle(i);        %elbow angle rotated

    for j=1:size(Dcf_array,1)
        Dcf=Dcf_array(j);
        tensor=tensor_array(j,i);
        t_angle=t_angle_array(j,i);
        %to plot the arm
        arm_plot=0;
        arm_plot(1,1)=arm_xini;
        arm_plot(2,1)=arm_yini;
        arm_plot(1,2)=arm_xini+La*cos(phi_shoulder);
        arm_plot(2,2)=arm_yini+La*sin(phi_shoulder);
        arm_plot(1,3)=arm_plot(1,2)+Lf*cos(phi_elbow);
        arm_plot(2,3)=arm_plot(2,2)+Lf*sin(phi_elbow);
        arm_plot(1,4)=arm_plot(1,3)+Lh*cos(phi_elbow);
        arm_plot(2,4)=arm_plot(2,3)+Lh*sin(phi_elbow);


        %to show the point of the arm clamping 
        alpha_Dca_pos=phi_shoulder+pi;
        dca_posx=forearm_xini+Dca*cos(alpha_Dca_pos);
        dca_posy=forearm_yini+Dca*sin(alpha_Dca_pos);

        %to show the point of the forearm clamping 
        alpha_Dcf_pos=phi_elbow;
        dcf_posx=forearm_xini+Dcf*cos(alpha_Dcf_pos);
        dcf_posy=forearm_yini+Dcf*sin(alpha_Dcf_pos);

        %to show the tensor ubication
        teta_tensor=phi_shoulder+t_angle;
        plot_tensor=[];
        plot_tensor(1,1)=dca_posx;
        plot_tensor(2,1)=dca_posy;
        plot_tensor(1,2)=dca_posx+tensor*cos(teta_tensor);
        plot_tensor(2,2)=dca_posy+tensor*sin(teta_tensor);

        %to show center of mass of the forearm
        Cmf_posx=forearm_xini+Dfcm*cos(phi_elbow);
        Cmf_posy=forearm_yini+Dfcm*sin(phi_elbow);

        %to show center of mass of the hand
        Cmh_posx=forearm_xini+Dd*cos(phi_elbow);
        Cmh_posy=forearm_yini+Dd*sin(phi_elbow);


        hold on
        plot(arm_plot(1,1:2),arm_plot(2,1:2),'-r','LineWidth',3)
        plot(arm_plot(1,2:3),arm_plot(2,2:3),'-b','LineWidth',3)
        plot(arm_plot(1,3:4),arm_plot(2,3:4),'-c','LineWidth',3)
        plot(dca_posx,dca_posy,'xk','MarkerSize',10);
        plot(dcf_posx,dcf_posy,'xr','MarkerSize',10);
        plot(plot_tensor(1,:),plot_tensor(2,:),'g')
        plot(Cmf_posx,Cmf_posy,'ks','MarkerSize',10);
        plot(Cmh_posx,Cmh_posy,'rs','MarkerSize',10);

        xlim([-(La+Lf+2*Lh)/2,(La+Lf+2*Lh)/2]);
        ylim([-(La+Lf+Lh),Lh]);
    end
end

end