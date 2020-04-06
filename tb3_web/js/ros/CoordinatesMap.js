//get coordinates_Map data
var map = document.getElementById("coordinates_Map");
var ctx = map.getContext("2d");
//get player data
var view = document.getElementById("player");
var view_button = document.getElementById("Camera-View");
var Camera = document.getElementById("View-or-Map");

var mouse_clicked = false;
var angle_offset = 0;
var mouse_angle = 0;
var buffer = []; // all designations coordinate's buffer
var coordinate = []; //push to robot's coordinate
var current = []; //current coordinate
var button; //mosue press the botton
var offective = [[60, 580], [64, 448]]; //located inside the competition venue
var origin = [320, 448]; //robot starting position
var mouse_clicked_range = false; //mouse position is inside the competition venue
var all_del = false; //ctrl and mouse button is 2 =>All Delete
var angle_buffer = mouse_angle; //when the mouse leave,mouse_angle need restore



function Norm_Angle(angle) { //confirm angle range is -360 ~ 360
    angle = angle * 180 / Math.PI //radius to degrees
    while (angle <= -360 || angle >= 360) {
        if (angle <= -360) {
            angle += 360;
        } else if (angle >= 360) {
            angle -= 360;
        }
    }
    angle = angle / 180 * Math.PI; //degrees to radius
    return angle;
}

function painting(mouse_x, mouse_y, mouse_angle ,color =false) { //painting coordinate in the webpage
 if (view.src.slice(-14) == "img/ground.png") {
        line_x = mouse_x - 20 * Math.sin(mouse_angle); //set line is up and meets the Right-hand rule
        line_y = mouse_y - 20 * Math.cos(mouse_angle);
        ctx.beginPath(); //start setting painting path
        ctx.lineWidth = 2;
        (color) ?
            (ctx.strokeStyle = "rgb(255, 0, 0)") : (ctx.strokeStyle = "rgb(55, 0, 255)")
        ctx.arc(mouse_x, mouse_y, 15, 0, 2 * Math.PI);
        ctx.moveTo(mouse_x, mouse_y);
        ctx.lineTo(line_x, line_y);
        ctx.stroke(); //start painting
        ctx.closePath(); //end painting
    }
}
function All_paint() { //data in the paint buffer
    for (let i = 0; i < buffer.length; i++) {
        painting(buffer[i][0], buffer[i][1], buffer[i][2]);
    }
}

function robot_Norm(data) { //convert data to robot specifications
    (data[1] != origin[1]) ?
        (x = parseFloat(((origin[1] - data[1]) / 128).toFixed(2))) : (x = 0.0);
    (data[0] != origin[0]) ?
        (y = parseFloat(((origin[0] - data[0]) / 130).toFixed(2))) : (y = 0.0);

    ang_ = Math.round(data[2] * 180 / Math.PI);
    if (ang_ == 0) {
        ang = 0.0;
    } else if (ang_ > 0 && ang_ <= 180 || ang_ > -180 && ang_ < 0) {
        ang = ang_;
    } else if (ang_ <= -180 && ang_ > -360) {
        ang = 360 + ang_;
    } else if (ang_ > 180 && ang_ < 360) {
        ang = ang_ - 360;
    }
    return [x,y,ang];
}
function Map_control() { //map send button's action when pressed
    if (view.src.slice(-14) == "img/ground.png") {

        coordinate = [];

        for (i = 0; i < buffer.length; i++) {
            data = robot_Norm(buffer[i]);
            coordinate.push(data);
        }
        // console.log(coordinate);

        coordToX = [];
        coordToY = [];
        coordToZ = [];

        for(i=0;i<buffer.length;i++){
            coordToX.push(coordinate[i][0]);
            coordToY.push(coordinate[i][1]);
            coordToZ.push(coordinate[i][2]);
        }
        console.log(" ");
        console.log("X");
        console.log(coordToX);
        console.log("Y");
        console.log(coordToY);
        console.log("Z")
        console.log(coordToZ);
        console.log(" ");

        PublishTopicmap(coordToX,coordToY,coordToZ);

    }
    else {
        console.log("Don't touch me");
    }
}


map.addEventListener("mousedown", function (e) {
    if (view.src.slice(-14) == "img/ground.png") {
        if (e.button == 0) {
            button = 0;
            mouse_clicked = true;
            mouse_x = e.offsetX;
            mouse_y = e.offsetY;

            mouse_x >= offective[0][0] && mouse_x < offective[0][1] && //confirm mouse position is inside the competition venue
                mouse_y >= offective[1][0] && mouse_y <= offective[1][1] ?
                (mouse_clicked_range = true) : (mouse_clicked_range = false);

            if (mouse_clicked_range && button == 0) {
                ctx.clearRect(0, 0, map.width, map.height); //clear canvas
                All_paint();
                painting(mouse_x, mouse_y, mouse_angle);
                current[0] = mouse_x;
                current[1] = mouse_y;
                current[2] = mouse_angle;
                angle_buffer = mouse_angle;
                map.addEventListener("mousemove", function (event) {
                    if (mouse_clicked && button == 0 && mouse_clicked_range) {
                        ctx.clearRect(0, 0, map.width, map.height);
                        All_paint();
                        let move_x = event.offsetX;
                        mouse_angle = Norm_Angle(angle_offset + (mouse_x - move_x) / 180 * Math.PI);
                        painting(mouse_x, mouse_y, mouse_angle);
                        current[2] = mouse_angle;
                    }
                })
            }
        } else if (e.button == 2) {
            button = 2;
            mouse_clicked = true;
            all_del = e.ctrlKey;
        } else {
            console.log("Please don't touch the strange button, ok?");
        }
    }
});

map.addEventListener("mouseup", function (e) {
    mouse_clicked = false;
    if (button == 0 && mouse_clicked_range) {
        print_data = robot_Norm(current);
        console.log(print_data);
        angle_offset = mouse_angle;
        buffer.push(current);
        current = [];
    } else if (button == 2) {
        all_del ?
            (buffer = [], console.log("All Delete")) : (buffer.pop(), console.log("One Delete"));
        ctx.clearRect(0, 0, map.width, map.height);
        All_paint();
    }
});
map.addEventListener("mouseleave", function (e) {
    mouse_clicked = false;
    mouse_angle = angle_buffer;
    ctx.clearRect(0, 0, map.width, map.height);
    All_paint();
});
