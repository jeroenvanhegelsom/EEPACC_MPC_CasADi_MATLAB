dir = '...\PACC_visualization\Assets\VisData';

useCaseNum = OPTsettings.useCaseNum;

writematrix(FBMPCsol.s_opt, [dir,'\UC',num2str(useCaseNum),'_s_FBMPC.txt']) 
writematrix(ABMPCsol.s_opt, [dir,'\UC',num2str(useCaseNum),'_s_ABMPC.txt']) 
writematrix(BLMPCsol.s_opt, [dir,'\UC',num2str(useCaseNum),'_s_BLMPC.txt']) 

writematrix(FBMPCsol.v_opt, [dir,'\UC',num2str(useCaseNum),'_v_FBMPC.txt']) 
writematrix(ABMPCsol.v_opt, [dir,'\UC',num2str(useCaseNum),'_v_ABMPC.txt']) 
writematrix(BLMPCsol.v_opt, [dir,'\UC',num2str(useCaseNum),'_v_BLMPC.txt']) 

writematrix(FBMPCsol.P_opt, [dir,'\UC',num2str(useCaseNum),'_P_FBMPC.txt']) 
writematrix(ABMPCsol.P_opt, [dir,'\UC',num2str(useCaseNum),'_P_ABMPC.txt']) 
writematrix(BLMPCsol.P_opt, [dir,'\UC',num2str(useCaseNum),'_P_BLMPC.txt']) 

writematrix(FBMPCsol.E_opt, [dir,'\UC',num2str(useCaseNum),'_E_FBMPC.txt']) 
writematrix(ABMPCsol.E_opt, [dir,'\UC',num2str(useCaseNum),'_E_ABMPC.txt']) 
writematrix(BLMPCsol.E_opt, [dir,'\UC',num2str(useCaseNum),'_E_BLMPC.txt']) 

writematrix(OPTsettings.s_tv, [dir,'\UC',num2str(useCaseNum),'_s_TV.txt']) 

writematrix(FBMPCsol.DistHor, [dir,'\UC',num2str(useCaseNum),'_DistHor_FBMPC.txt']) 
writematrix(ABMPCsol.DistHor, [dir,'\UC',num2str(useCaseNum),'_DistHor_ABMPC.txt']) 

writematrix(Ts, [dir,'\UC',num2str(useCaseNum),'_Ts.txt']) 