import roboticstoolbox as rtb 
import spatialmath as sm

sawyer = rtb.models.DH.Sawyer()

ellipsoid = rtb.backend.PyPlot.EllipsePlot(sawyer, etype="v")
sawyer.plot_ellipse(ellipsoid)