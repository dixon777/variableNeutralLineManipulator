from PySide2 import QtCore, QtGui, QtWidgets
from generated_widget import Ui_Dialog

class MainWidgetWrapper(Ui_Dialog):
    def setupUi(self, Dialog):
        super().setupUi(Dialog)
        QtCore.QObject.connect(self.generateButton, QtCore.SIGNAL("clicked()"), self.generate)

    def generate(self):
        import cadquery_disk_generator        
        param = {
            "outerDiameter": float(self.outerDiameterLineEdit.text()) if self.outerDiameterLineEdit.text() else 0,
            "length": float(self.lengthLineEdit.text()) if self.lengthLineEdit.text() else 0,
            "bottomCurvatureRadius": float(self.bottomCurvatureRadiusLineEdit.text()) if self.bottomCurvatureRadiusLineEdit.text() else None,
            "topCurvatureRadius": float(self.topCurvatureRadiusLineEdit.text()) if self.topCurvatureRadiusLineEdit.text() else None,
            "topCurvatureRelativeOrientation": float(self.topCurvatureRelativeOrientationLineEdit.text()) if self.topCurvatureRelativeOrientationLineEdit.text() else 0.0,
            "centreHoleDiameter": float(self.centreHoleDiameterLineEdit.text()) if self.centreHoleDiameterLineEdit.text() else None,

            "tendonDistFromAxis": float(self.tendonDistanceFromAxisLineEdit.text()) if self.tendonDistanceFromAxisLineEdit.text() else None,
            "tendonGuideDiameter": float(self.tendonGuideDiameterLineEdit.text()) if self.tendonGuideDiameterLineEdit.text() else None,
            "numTendonGuides": int(self.numberOfTendonGuidesLineEdit.text()) if self.numberOfTendonGuidesLineEdit.text() else None,
        } 
        
        
        res_obj, errors, warnings = cadquery_disk_generator.construct_disk(**param)
        if res_obj:
            cadquery_disk_generator.export(res_obj,filename=self.exportPathLineEdit.text() if self.exportPathLineEdit.text() else None)
        
        error_warning_text = ""
        if errors:
            error_warning_text += "Errors\n"
            for i, e in enumerate(errors):
                error_warning_text += f"{i}. {e}"
        
        
        if warnings:
            error_warning_text += "Warnings\n"
            for i, e in enumerate(warnings):
                error_warning_text += f"{i}. {e}"
        
        self.errorTextbox.setText(error_warning_text)
            
        
        
        

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = MainWidgetWrapper()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())