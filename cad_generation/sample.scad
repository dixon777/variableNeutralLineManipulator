difference(){
    cylinder(h=5, d=5, center=true, $fn=300);
    rotate(a=45.0, v=[0, 0, 1]){
        difference(){
            translate(v=[0, 0, 1.6224989991991992]){
                cube(size=[6, 6, 3.5], center=true);
            };
            translate(v=[0, 0, -1.5]){
                rotate(a=90, v=[0, 1, 0]){
                    cylinder(h=8, r=4, center=true, $fn=300);
                };
            };
        };
    };
    translate(v=[1.8, 0.0, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[1.2727922061357857, 1.2727922061357857, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[1.1021821192326179e-16, 1.8, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[-1.2727922061357855, 1.2727922061357857, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[-1.8, 2.2043642384652358e-16, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[-1.272792206135786, -1.2727922061357855, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[-3.3065463576978537e-16, -1.8, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[1.2727922061357853, -1.272792206135786, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
    translate(v=[1.8, -4.4087284769304716e-16, 0]){
        cylinder(h=6, d=1, center=true, $fn=100);
    };
};
