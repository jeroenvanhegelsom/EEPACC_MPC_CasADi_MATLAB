function PlotAnimFrame_MPC(k,s_opt,v_opt,s_opt_pred,v_opt_pred,s_tv_est,v_tv_est,EVtrajPlot,EVpredPlot,TVtrajPlot,TVpredPlot)
    set(EVtrajPlot,'Xdata',s_opt(1:k))
    set(EVtrajPlot,'Ydata',3.6*v_opt(1:k))
    set(EVpredPlot,'Xdata',s_opt_pred)
    set(EVpredPlot,'Ydata',3.6*v_opt_pred)
    set(TVtrajPlot,'Xdata',s_tv_est(1))
    set(TVtrajPlot,'Ydata',3.6*v_tv_est(1))
    set(TVpredPlot,'Xdata',s_tv_est)
    set(TVpredPlot,'Ydata',3.6*v_tv_est)
    gif
end

