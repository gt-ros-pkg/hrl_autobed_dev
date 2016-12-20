var AutoBed = AutoBed || {};

AutoBed.log = function (txt) {
    var logDiv =  document.getElementById('log');
    logDiv.innerHTML = txt;
}

AutoBed.start = function () {
    AutoBed.SERVER = window.location.host.split(':')[0];//Get server from URL in browser
    AutoBed.PORT = '8889';//Must match port on which rosbridge is being served
    AutoBed.ros = new ROSLIB.Ros({url: 'ws://'+ AutoBed.SERVER + ':'+ AutoBed.PORT});
    AutoBed.ros.on('close', function(e) {
        AutoBed.log("Cannot connect to AutoBed.")
        console.log("Cannot connect to AutoBed at "+ AutoBed.SERVER + ":"+ AutoBed.PORT + ".");
        });

    AutoBed.ros.on('error', function(e) {
        AutoBed.log("Error with connection to AutoBed!");
        });

    AutoBed.ros.on('connection', function(e) {
        AutoBed.log("Connected to AutoBed.");
        AutoBed.rosInit();
        });
};

AutoBed.rosInit = function () {
    //Register a publisher for differential commands to the AutoBed
    AutoBed.controlsPub = new ROSLIB.Topic({
        ros: AutoBed.ros,
        name: "/abdin1",
        messageType: "std_msgs/String"
    });
    AutoBed.controlsPub.advertise();

    AutoBed.sendDiffCmd = function (string) {
        if (AutoBed.send) {
            AutoBed.controlsPub.publish({'data':string});
            AutoBed.diffTimer = setTimeout(function () {
                    AutoBed.sendDiffCmd(string)}
                    , 75)
        } else {
            clearTimeout(AutoBed.diffTimer);
        }
    };

    var setupButtonCBs = function (buttonElement) {
        buttonElement.addEventListener("mouseup", function(event){
                    AutoBed.send = false;
                    clearTimeout(AutoBed.diffTimer);
                    });
        buttonElement.addEventListener("mouseout", function(event){
                    AutoBed.send = false;
                    clearTimeout(AutoBed.diffTimer);
                    });
        buttonElement.addEventListener("mouseleave", function(event){
                    AutoBed.send = false;
                    clearTimeout(AutoBed.diffTimer);
                    });
        buttonElement.addEventListener("mousedown", function(event) {
                    AutoBed.send = true;
                    AutoBed.diffTimer = setTimeout(function () {
                        AutoBed.sendDiffCmd(event.target.value);
                        }, 50);
                    });
    };

    setupButtonCBs(document.getElementById("head-up"));
    setupButtonCBs(document.getElementById("bed-up"));
    setupButtonCBs(document.getElementById("legs-up"));
    setupButtonCBs(document.getElementById("head-down"));
    setupButtonCBs(document.getElementById("bed-down"));
    setupButtonCBs(document.getElementById("legs-down"));

}
