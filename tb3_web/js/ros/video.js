//===========================================================================
//new

function Camera_Switch(value) {
    var view = document.getElementById("Camera-View");
    if (view.checked) {
        viewflag = value;
    }else {
        viewflag = 'off' ;
    }
}

function Open_Camera() {
    var Camera = document.getElementById("View-or-Map");
    var video = document.getElementById("player");
    var view = document.getElementById("Camera-View");
    var mode = parseInt(document.getElementById('HSVSelect').value);
    let camera1 = document.getElementById('Camera1').checked;
    let camera2 = document.getElementById('Camera2').checked;

    if (camera2) {
        if (mode == 0) mode = 5;
        if (mode == 1) mode = 6;
        if (mode == 2) mode = 7;
    }

    Camera_Switch();
    if (view.checked) {

        //set default
        if(document.getElementById('Camera-src').checked){
            viewflag = 'src';
        }else if (document.getElementById('Camera-Mask').checked){
            viewflag = 'mask';
        }else if (document.getElementById('Camera-Monitor').checked){
            viewflag = 'monitor';
        }

        if (viewflag == 'src') {
            video.style.height = "100%";
            video.style.top = "0%";
            if (camera1) {
                setTimeout(function () { video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/src" }, 100);
            }
            if (camera2) {
                video.style.height = "100%";
                video.style.top = "0%";
                setTimeout(function () { video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/src2" }, 100);
            }
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/src";
        } else if (viewflag == 'mask') {
            //video.position="relative";
            video.style.top = "40%"
            video.style.height = "60%";
            console.log('mask : '+mode);
            switch (mode) {
                case 0:
                    Pub_VideoMode(0);
                    break;
                case 1:
                    Pub_VideoMode(1);
                    break;
                case 2:
                    Pub_VideoMode(2);
                    break;
                case 3:
                    Pub_VideoMode(3);
                    break;
                case 4:
                    Pub_VideoMode(4);
                    break;
                case 5:
                    Pub_VideoMode(5);
                    break;
                case 6:
                    Pub_VideoMode(6);
                    break;
                case 7:
                    Pub_VideoMode(7);
                    break;
            }

            if (camera1) {
                setTimeout(function () { video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/mask" }, 100);
            }
            if (camera2) {
                video.style.height = "60%";
                video.style.top = "40%";
                setTimeout(function () { video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/mask2" }, 100);
            }
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/mask";
        } else if (viewflag == 'monitor') {
            video.style.top = "40%"
            video.style.height = "60%";
            if (camera1) {
                setTimeout(function () { video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/monitor" }, 100);
            }
            if (camera2) {
                video.style.height = "60%";
                video.style.top = "40%";
                setTimeout(function () { video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/monitor2" }, 100);
            }
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/monitor";
        } else if (viewflag == 'off') {
            video.src = "img/black.png";
        }
    } else {
        video.style.top = "0%"
        video.style.height = "100%";
        video.src = "img/ground.png";
    }

    /*if (Topic_VideoMode_Flag) {
        if (viewflag == 'mask') {
            console.log(viewflag + ' mode : ' + mode);
        } else {
            console.log(viewflag);
        }
    }*/
}




