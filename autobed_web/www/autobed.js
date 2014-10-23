var AutoBed = AutoBed || {};

AutoBed.log = function (txt) {
    var logDiv =  document.getElementById('log');
    logDiv.innerHTML = txt;
}

AutoBed.start = function () {
    AutoBed.SERVER = window.location.host.split(':')[0];//Get server from URL in browser
    AutoBed.PORT = '2206';//Must match port on which rosbridge is being served
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
                    , 250)
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
/*
    //Handle the success/failure response from saving a pose
    AutoBed.saveResponseCB = function (resp) {
        if (resp.success) {
            AutoBed.log("Pose Saved.");
        } else {
            AutoBed.log("FAILED to save pose.");
            }
    };

    //Service to request saving current pose
    AutoBed.saveServiceClient = new ROSLIB.Service({
        ros: AutoBed.ros,
        name: '/add_autobed_config',
        serviceType: 'autobed_engine/update_autobed_config' 
    });

    // Process button-click to save pose
    AutoBed.saveButtonCB = function (event) {
        var req = { 'config': document.getElementById('record-name').value };
        if (req['config'] === '') {
            alert("Please enter a name for the pose");
            AutoBed.log("Please enter a name for the pose");
            return;
        } else if (req['config'].indexOf(":") >= 0 ||
                   req['config'].indexOf("-") >= 0 ||
                   req['config'].indexOf("\t") >= 0 ) {
            alert("Name of pose should not contain ':', '-', or tabs.");
            AutoBed.log("Name of pose should not contain ':', '-', or tabs.");
            return;
        }
        AutoBed.saveServiceClient.callService(req, AutoBed.saveResponseCB);
    }

    // Register handler for save pose button
    var saveButton = document.getElementById('save-pose');
    saveButton.addEventListener('click', AutoBed.saveButtonCB);

    //Handle the success/failure response from saving a pose
    AutoBed.deleteResponseCB = function (resp) {
        if (resp.success) {
            AutoBed.log("Pose deleted.");
        } else {
            AutoBed.log("FAILED to delete pose.");
            }
    };

    //Service to request saving current pose
    AutoBed.deleteServiceClient = new ROSLIB.Service({
        ros: AutoBed.ros,
        name: '/delete_autobed_config',
        serviceType: 'autobed_engine/update_autobed_config' 
    });

    // Process button-click to save pose
    AutoBed.deleteButtonCB = function (event) {
        var sel = document.getElementById('pose-select');
        var req = { 'config' : AutoBed.poses[sel.value] };
        AutoBed.deleteServiceClient.callService(req, AutoBed.deleteResponseCB);
    }

    // Register handler for save pose button
    var deleteButton = document.getElementById('delete-pose');
    deleteButton.addEventListener('click', AutoBed.deleteButtonCB);

    // Callback for subscription to known AutoBed poses
    AutoBed.posesCB = function (stringList) {
        // Save pose names and positions
        AutoBed.poses = {};
        for (idx in stringList.data) {
            AutoBed.poses[idx] = stringList.data[idx];
        }
        var sel = document.getElementById('pose-select');
        // Clear out existing options
        for (var i = sel.length-1; i>=0; i--) {
            sel.remove(i);
        }
        // Re-populate select box
        for (idx in AutoBed.poses) {
            sel.add(new Option(AutoBed.poses[idx], idx));
        }
    }
    
    // Subscribe to recorded/known poses
    AutoBed.posesSub = new ROSLIB.Topic({
        ros: AutoBed.ros,
        name: '/abdout1',
        messageType: '/hrl_msgs/StringArray'
    });
    AutoBed.posesSub.subscribe(AutoBed.posesCB);

    // Publish pose commands to bed
    AutoBed.poseCmdPub = new ROSLIB.Topic({
        ros: AutoBed.ros,
        name: "/abdin1",
        messageType: "std_msgs/String"
    });
    AutoBed.poseCmdPub.advertise();
    
    // Callback for send pose button
    AutoBed.sendPose = function (event) {
        var sel = document.getElementById('pose-select');
        var msg = { 'data' : AutoBed.poses[sel.value] };
        AutoBed.poseCmdPub.publish( msg );
    };

    // Attach button callback to send pose button
    var sendButton = document.getElementById('send-pose');
    sendButton.addEventListener("click", AutoBed.sendPose)
    */
}


