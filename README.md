## SofaGLFW
This SOFA plugin brings a simple GUI based on GLFW (a spiritual successor of Glut).

It only needs SofaGUICommon, SofaBaseVisual and Sofa.GL as dependencies.
Integration of GLFW is automatic (automatic fetching and integration with cmake), and linked statically (does not need a glfw.dll to be shipped with)

This GUI is launchable with the standard runSofa (with the parameter "-g glfw"), or can be used with a (provided) stand-alone executable (which needs much less dependencies than runSofa)

Lastly, this GUI was designed to support multiple windows in the same time and multiple simulations. So when multiple simulations is possible is the future, it should be easy to modify the code to support this feature.
And multiple windows could be based on the fact having multiple Camera in the scene (feature not implemented yet)
