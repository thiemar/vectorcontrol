/*
Copyright (C) 2014-2015 Thiemar Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/* Bah */
NodeList.prototype.forEach = Array.prototype.forEach;
HTMLCollection.prototype.forEach = Array.prototype.forEach;

var ws, nodeData = {}, deviceCurrentCharts = {}, deviceSpeedCharts = {},
    deviceAccelPowerCharts = {}, deviceVoltageTempCharts = {},
    deviceAnimationCallbacks = {}, deviceOutputVoltageCharts = {},
    deviceAirspeedCharts = {}, deviceLoadCharts = {}, deviceThrustCharts = {},
    activeESCs = {}, nodeLastStatus = {},
    escRawSetpointFunctions = [], lastUpdate = null, highestEscIndex = -1,
    enumerationActive = false, lastEnumeratedNodeId = -1;

function getParamValue(param) {
    if (param.real_value !== undefined) {
        return param.real_value;
    } else if (param.integer_value !== undefined) {
        return param.integer_value;
    } else if (param.string_value !== undefined) {
        return param.string_value;
    } else {
        return null;
    }
}

function connect() {
    ws = new WebSocket("ws://" + window.location.host + "/can");
    lastUpdate = (new Date()).valueOf();

    /*
    Handle new messages from the server. These generally map to messages
    received over the CAN interface, although the Python backend aggregates
    some of the high-frequency messages to reduce load.
    */
    ws.onmessage = function(event) {
        var message = JSON.parse(event.data), nodeUi, content, temp, input;

        if (message.node_id === undefined) {
            return;
        }

        lastUpdate = (new Date()).valueOf();

        /*
        Check for the existence of an interface for this node. If not found,
        clone the template interface.
        */
        nodeUi = document.getElementById("device-" + message.node_id);
        nodeLastStatus[message.node_id] = (new Date()).valueOf();

        /* Process message data */
        if (message.datatype == "uavcan.protocol.GetNodeInfo") {
            if (!nodeUi) {
                createNodeUi(message);
            }

        } else if (message.datatype == "uavcan.protocol.NodeStatus" &&
                   nodeUi) {
            /* Node status -- display the new uptime and status code */
            nodeUi.querySelector("span.node-uptime").textContent =
                message.payload.uptime_sec;
            nodeUi.querySelector("span.node-health").textContent = {
                0: "ok",
                1: "warning",
                2: "error",
                3: "critical"
            }[message.payload.health];
            nodeUi.querySelector("span.node-mode").textContent = {
                0: "operational",
                1: "intialization",
                2: "maintenance",
                3: "software update",
                7: "offline"
            }[message.payload.mode];
        } else if (message.datatype == "uavcan.protocol.param.GetSet" &&
                   nodeUi) {
            /*
            Parameter value update -- set the corresponding input to the new
            value, and update the chart scaling if necessary.
            */
            input = nodeUi.querySelector("input[name='" + message.payload.name + "']");

            input.setAttribute("min", getParamValue(message.payload.min_value));
            input.setAttribute("max", getParamValue(message.payload.max_value));

            if (message.payload.value.real_value !== undefined) {
                input.value = getParamValue(message.payload.value).toPrecision(4);
                input.classList.add("dtype-real");
            } else {
                input.value = getParamValue(message.payload.value);
                input.classList.add("dtype-integer");
            }

            if (false) {
                updateCurrentChart(nodeUi, nodeData[message.node_id] || []);
                updateSpeedChart(nodeUi, nodeData[message.node_id] || []);
                updateOutputVoltageChart(nodeUi,
                                         nodeData[message.node_id] || []);
            }
        } else if ((message.datatype == "thiemar.equipment.esc.Status" ||
                   message.datatype == "uavcan.equipment.esc.Status" ||
                   message.datatype == "uavcan.equipment.air_data.TrueAirspeed" ||
                   message.datatype == "uavcan.equipment.air_data.IndicatedAirspeed" ||
                   message.datatype == "uavcan.equipment.hardpoint.Status") &&
                   nodeUi) {
            /*
            Measurement data -- add it to the measurement array, removing old
            data if the total length is more than 600 samples
            (30 seconds * 20 Hz). Update the charts once done.
            */
            if (!nodeData[message.node_id]) {
                nodeData[message.node_id] = {};
            }
            if (!nodeData[message.node_id][message.datatype]) {
                nodeData[message.node_id][message.datatype] = [];
            }
            nodeData[message.node_id][message.datatype].push(message.payload);
            if (nodeData[message.node_id][message.datatype].length > 600) {
                nodeData[message.node_id][message.datatype] =
                    nodeData[message.node_id][message.datatype].slice(
                        nodeData[message.node_id][message.datatype].length - 600);
            }

            if (!deviceAnimationCallbacks[message.node_id]) {
                deviceAnimationCallbacks[message.node_id] =
                    requestAnimationFrame(function () {
                        updateCharts(nodeUi, nodeData[message.node_id]);
                    });
            }

            if (message.datatype == "thiemar.equipment.esc.Status" ||
                    message.datatype == "uavcan.equipment.esc.Status") {
                ensureESCActive(message);
            }
        } else if (message.datatype == "uavcan.protocol.enumeration.Indication" &&
                message.payload.parameter_name == "esc_index" &&
                message.node_id != lastEnumeratedNodeId) {
            /*
            Update the ESC index, apply the configuration and restart the node
            */
            highestEscIndex++;
            ws.send(JSON.stringify({
                node_id: message.node_id,
                datatype: "uavcan.protocol.param.GetSet",
                payload: {
                    index: 0,
                    name: "esc_index",
                    value: {integer_value: highestEscIndex}
                }
            }));
            ws.send(JSON.stringify({
                node_id: message.node_id,
                datatype: "uavcan.protocol.param.ExecuteOpcode",
                payload: {opcode: 0, argument: 0}
            }));
            ws.send(JSON.stringify({
                datatype: "uavcan.equipment.indication.BeepCommand",
                payload: {duration: 0.25, frequency: 440.0}
            }));

            window.setTimeout(function() {
                ws.send(JSON.stringify({
                    node_id: message.node_id,
                    datatype: "uavcan.protocol.RestartNode",
                    payload: {magic_number: 0xACCE551B1E}
                }));
            }, 500);

            lastEnumeratedNodeId = message.node_id;
        }
    }

    /* Send an initial message to trigger the initial fetch */
    ws.onopen = function(event) {
        ws.send("{}");
    }

    /*
    Automatic reconnect on timeout -- check every second and if no messages
    have been received for the last two seconds, close the socket and try
    again.

    Also send an empty message to the server to keep that end alive.
    */
    ws_timeout = window.setInterval(function() {
        if ((new Date()).valueOf() - lastUpdate > 2000.0) {
            window.clearInterval(ws_timeout);
            ws.close();
            connect();
        } else {
            ws.send("{}");
        }

        var nodes, i, nodeId, cutoffTime;

        nodes = document.querySelectorAll(".device");
        cutoffTime = (new Date()).valueOf() - 3000.0;

        for (i = 0; i < nodes.length; i++) {
            if (nodes[i].id && nodes[i].id.indexOf("device-") === 0) {
               nodeId = parseInt(nodes[i].id.split("-")[1], 10);

                if (nodeLastStatus[nodeId] < cutoffTime) {
                    nodes[i].remove();
                }
            }
        }
    }, 1000.0);
}


function createNodeUi(message) {
    var deviceName = message.payload.name.replace(/\./g, "_");

    content = document.getElementById("content");
    nodeUi = document.querySelector(".hidden.device-template-" + deviceName).cloneNode(true);
    nodeUi.id = "device-" + message.node_id;
    nodeUi.querySelector("span.node-id").textContent = message.node_id;
    nodeUi.querySelector("span.node-name").textContent =
        message.payload.name;
    nodeUi.classList.remove("hidden");
    content.appendChild(nodeUi);

    if (nodeUi.classList.contains("device-template-com_thiemar_s2740vc-v1") ||
            nodeUi.classList.contains("device-template-org_pixhawk_px4esc-v1")) {
        setupSpeedChart(nodeUi);
        setupCurrentChart(nodeUi);
        setupVoltageTempChart(nodeUi);
        setupOutputVoltageChart(nodeUi);
    } else if (nodeUi.classList.contains("device-template-com_thiemar_p7000d-v1")) {
        setupAirspeedChart(nodeUi);
    } else if (nodeUi.classList.contains("device-template-com_thiemar_loadsensor-v1")) {
        setupLoadChart(nodeUi);
    }
}


function ensureESCActive(message) {
    var escIdx = message.payload.esc_index,
        escUi = document.getElementById("control-esc-" + escIdx),
        uiContainer =
            document.getElementById("control-esc-template").parentNode,
        childIdx;

    activeESCs[message.payload.esc_index] = (new Date()).valueOf();

    if (!escUi) {
        escUi = document.getElementById("control-esc-template").cloneNode(true);
        escUi.id = "control-esc-" + escIdx;
        escUi.querySelector("span.esc-index").textContent = "ESC " +
            message.payload.esc_index;
        escUi.classList.remove("hidden");

        /* Insert the node in ESC index order */
        for (var i = 1; i < uiContainer.children.length; i++) {
            childIdx = parseInt(uiContainer.children[i].id.split("-")[2], 10);
            if (childIdx > escIdx) {
                uiContainer.insertBefore(escUi, uiContainer.children[i]);
                break;
            }
        }

        if (i == uiContainer.children.length) {
            uiContainer.appendChild(escUi);
        }
    }
}


function setupVoltageTempChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.voltagetemp-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y0 = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 27.0]);

    result.y1 = d3.scale.linear()
        .range([height, 0.0])
        .domain([-40.0, 85.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickFormat("")
        .tickSize(-height, 0, 0);
    result.yAxis0 = d3.svg.axis()
        .scale(result.y0)
        .orient("left");
    result.yAxis1 = d3.svg.axis()
        .scale(result.y1)
        .orient("right");

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis);

    result.chart.append("g")
        .attr("class", "y0 axis")
        .call(result.yAxis0)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Bus voltage (V)");

    result.chart.append("g")
        .attr("class", "y1 axis")
        .attr("transform", "translate(" + width + ", 0)")
        .call(result.yAxis1)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", 40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Temperature (°C)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "vbus")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "temperature")
        .attr('clip-path', 'url(#clip)');

    deviceVoltageTempCharts[parseInt(device.id.split("-")[1], 10)] = result;
}


function setupCurrentChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.current-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([-40.0, 40.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickFormat("")
        .tickSize(-height, 0, 0);
    result.yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickSize(-width, 0, 0);

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis);

    result.chart.append("g")
        .attr("class", "y axis")
        .call(result.yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Current draw (A)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "current-setpoint")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "current-id")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "current-iq")
        .attr('clip-path', 'url(#clip)');

    deviceCurrentCharts[parseInt(device.id.split("-")[1], 10)] = result;
}


function setupSpeedChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.speed-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 10000.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickFormat("")
        .tickSize(-height, 0, 0);
    result.yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickSize(-width, 0, 0);

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis);

    result.chart.append("g")
        .attr("class", "y axis")
        .call(result.yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Speed (rpm)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "speed-actual")
        .attr('clip-path', 'url(#clip)');

    deviceSpeedCharts[parseInt(device.id.split("-")[1], 10)] = result;
}


function setupOutputVoltageChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.output-voltage-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 50};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([-30.0, 30.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickSize(-height, 0, 0);
    result.yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickSize(-width, 0, 0);

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis)
    .append("text")
        .attr("x", width / 2)
        .attr("y", 30)
        .style("text-anchor", "middle")
        .text("Time (s)");

    result.chart.append("g")
        .attr("class", "y axis")
        .call(result.yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Output voltage (V)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "voltage-vd")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "voltage-vq")
        .attr('clip-path', 'url(#clip)');

    deviceOutputVoltageCharts[parseInt(device.id.split("-")[1], 10)] = result;
}


function setupAirspeedChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.airspeed-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 50.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickFormat("")
        .tickSize(-height, 0, 0);
    result.yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickSize(-width, 0, 0);

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis);

    result.chart.append("g")
        .attr("class", "y axis")
        .call(result.yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Airspeed (m/s)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "airspeed-tas")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "airspeed-ias")
        .attr('clip-path', 'url(#clip)');

    deviceAirspeedCharts[parseInt(device.id.split("-")[1], 10)] = result;
}


function setupLoadChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.load-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y = d3.scale.linear()
        .range([height, 0.0])
        .domain([-5.0, 15.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
        .tickFormat("")
        .tickSize(-height, 0, 0);
    result.yAxis = d3.svg.axis()
        .scale(result.y)
        .orient("left")
        .tickSize(-width, 0, 0);

    result.chart = d3.select(svg)
        .attr("width", width + margin.left + margin.right)
        .attr("height", height + margin.top + margin.bottom)
        .append("g")
            .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

    result.chart.append("g")
        .attr("class", "x axis")
        .attr("transform", "translate(0, " + height + ")")
        .call(result.xAxis);

    result.chart.append("g")
        .attr("class", "y axis")
        .call(result.yAxis)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Load (N)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "load-n")
        .attr('clip-path', 'url(#clip)');

    deviceLoadCharts[parseInt(device.id.split("-")[1], 10)] = result;
}


function updateCharts(device, data) {
    var deviceId;
    deviceId = parseInt(device.id.split("-")[1], 10);
    deviceAnimationCallbacks[deviceId] = undefined;

    if (device.classList.contains("device-template-com_thiemar_s2740vc-v1") ||
            device.classList.contains("device-template-org_pixhawk_px4esc-v1")) {
        updateCurrentChart(deviceId, device, data);
        updateSpeedChart(deviceId, device, data);
        updateVoltageTempChart(deviceId, device, data);
        updateOutputVoltageChart(deviceId, device, data);
    } else if (device.classList.contains("device-template-com_thiemar_p7000d-v1")) {
        updateAirspeedChart(deviceId, device, data);
    } else if (device.classList.contains("device-template-com_thiemar_loadsensor-v1")) {
        updateLoadChart(deviceId, device, data);
    }
}


function updateVoltageTempChart(deviceId, device, data) {
    var seriesData, chart;

    chart = deviceVoltageTempCharts[deviceId];

    /* Vbus line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y0(d.voltage); });

    chart.chart.select(".vbus")
        .datum(data["uavcan.equipment.esc.Status"])
        .attr("d", seriesData);

    /* Temperature line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y1(d.temperature - 273.15); });

    chart.chart.select(".temperature")
        .datum(data["uavcan.equipment.esc.Status"])
        .attr("d", seriesData);
}


function updateCurrentChart(deviceId, device, data) {
    var current, chart, maxCurrent;

    chart = deviceCurrentCharts[deviceId];
    maxCurrent = parseFloat(device.querySelector("input[name=mot_i_max]").value) || 40.0;

    chart.y.domain([-maxCurrent, maxCurrent]);
    chart.yAxis.scale(chart.y);
    chart.chart.select(".y.axis").call(chart.yAxis);

    /* Setpoint line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.i_setpoint); });

    chart.chart.select(".current-setpoint")
        .datum(data["thiemar.equipment.esc.Status"])
        .attr("d", current);

    /* Id line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.i_dq[0]); });

    chart.chart.select(".current-id")
        .datum(data["thiemar.equipment.esc.Status"])
        .attr("d", current);

    /* Iq line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.i_dq[1]); });

    chart.chart.select(".current-iq")
        .datum(data["thiemar.equipment.esc.Status"])
        .attr("d", current);
}


function updateSpeedChart(deviceId, device, data) {
    var speed, chart, maxSpeed;

    chart = deviceSpeedCharts[deviceId];
    maxSpeed = parseFloat(device.querySelector("input[name=mot_kv]").value) *
               parseFloat(device.querySelector("input[name=mot_v_max]").value);

    if (isNaN(maxSpeed)) {
        maxSpeed = 10000.0;
    }

    chart.y.domain([-maxSpeed * 1.1, maxSpeed * 1.1]);
    chart.yAxis.scale(chart.y);
    chart.chart.select(".y.axis").call(chart.yAxis);

    /* Actual line */
    speed = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.rpm); });

    chart.chart.select(".speed-actual")
        .datum(data["uavcan.equipment.esc.Status"])
        .attr("d", speed);
}


function updateOutputVoltageChart(deviceId, device, data) {
    var voltage, chart, maxVoltage, consistency;

    chart = deviceOutputVoltageCharts[deviceId];
    maxVoltage = parseFloat(device.querySelector("input[name=mot_v_max]").value) || 27.0;

    chart.y.domain([-maxVoltage, maxVoltage]);
    chart.yAxis.scale(chart.y);
    chart.chart.select(".y.axis").call(chart.yAxis);

    /* Vd line */
    voltage = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.v_dq[0]); });

    chart.chart.select(".voltage-vd")
        .datum(data["thiemar.equipment.esc.Status"])
        .attr("d", voltage);

    /* Vq line */
    voltage = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.v_dq[1]); });

    chart.chart.select(".voltage-vq")
        .datum(data["thiemar.equipment.esc.Status"])
        .attr("d", voltage);
}


function updateAirspeedChart(deviceId, device, data) {
    var airspeed, chart;

    chart = deviceAirspeedCharts[deviceId];

    /* TAS line */
    airspeed = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.true_airspeed); });

    chart.chart.select(".airspeed-tas")
        .datum(data["uavcan.equipment.air_data.TrueAirspeed"])
        .attr("d", airspeed);

    /* IAS line */
    airspeed = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.indicated_airspeed); });

    chart.chart.select(".airspeed-ias")
        .datum(data["uavcan.equipment.air_data.IndicatedAirspeed"])
        .attr("d", airspeed);
}


function updateLoadChart(deviceId, device, data) {
    var load, chart;

    chart = deviceLoadCharts[deviceId];

    /* Load line */
    load = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.cargo_weight || d.payload_weight); });

    chart.chart.select(".load-n")
        .datum(data["uavcan.equipment.hardpoint.Status"])
        .attr("d", load);
}


function updateSetpoint() {
    var rawSetpoints = [],
        time = (new Date()).valueOf() * 0.001,  /* seconds */
        maxRawIdx = -1;

    for (var i = 0; i < 16; i++) {
        if (escRawSetpointFunctions[i]) {
            rawSetpoints.push(parseInt(escRawSetpointFunctions[i](time) *
                                       (8191.0 / 100.0), 10));
            maxRawIdx = i;
        } else {
            rawSetpoints.push(0.0);
        }
    }

    if (maxRawIdx >= 0) {
        ws.send(JSON.stringify({
            datatype: "uavcan.equipment.esc.RawCommand",
            payload: {
                cmd: rawSetpoints.slice(0, maxRawIdx + 1)
            }
        }));
    }
}


function selectAncestor(elem, selector) {
    elem = elem.parentNode;
    while (elem.parentNode != document) {
        if (elem.matches(selector)) {
            return elem;
        } else {
            elem = elem.parentNode;
        }
    }
    return null;
}


function setupEventListeners() {
    var content;

    content = document.getElementById("content");
    content.addEventListener("change", function(event) {
        var nodeUi;

        if (event.target.classList.contains("configuration")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement.parentElement;
            ws.send(JSON.stringify({
                node_id: parseInt(nodeUi.id.split("-")[1], 10),
                datatype: "uavcan.protocol.param.GetSet",
                payload: {
                    index: 0,
                    name: event.target.name,
                    value: (event.target.classList.contains("dtype-real") ?
                        {real_value: parseFloat(event.target.value)} :
                        {integer_value: parseInt(event.target.value, 10)})
                }
            }));
        } else if (event.target.name == "command") {
            escUi = event.target.parentElement.parentElement.parentElement;
            escUi.querySelectorAll("label.command")
                    .forEach(function(elem) {
                if (elem.classList.contains("command-" + event.target.value)) {
                    elem.classList.remove("hidden");
                } else {
                    elem.classList.add("hidden");
                }
            });
        }
    });

    content.addEventListener("click", function(event) {
        var nodeUi, escUi, escIndex, func, commandMode;

        if (event.target.classList.contains("apply-configuration")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement;
            ws.send(JSON.stringify({
                node_id: parseInt(nodeUi.id.split("-")[1], 10),
                datatype: "uavcan.protocol.param.ExecuteOpcode",
                payload: {opcode: 0, argument: 0}
            }));
            ws.send(JSON.stringify({
                node_id: parseInt(nodeUi.id.split("-")[1], 10),
                datatype: "uavcan.protocol.RestartNode",
                payload: {magic_number: 0xACCE551B1E}
            }));

            event.stopPropagation();
        } else if (event.target.classList.contains("esc-startstop")) {
            escUi = event.target.parentElement.parentElement.parentElement;
            escIndex = parseInt(escUi.id.split("-")[2], 10);

            if (event.target.value == "Start") {
                escUi.querySelectorAll("input, select").forEach(function(elem) {
                    elem.disabled = elem != event.target;
                });

                /*
                Freeze the setpoint command data and add it to the setpoint
                function array
                */
                func = makeEscSetpointFunction((new Date()).valueOf() * 0.001,
                                               escUi);
                escRawSetpointFunctions[escIndex] = func;

                event.target.value = "Stop";
            } else if (event.target.value == "Stop") {
                escUi.querySelectorAll("input, select").forEach(function(elem) {
                    elem.disabled = false;
                });

                event.target.value = "Start";
                escRawSetpointFunctions[escIndex] = null;
            }

            event.stopPropagation();
        } else if (event.target.classList.contains("esc-beep")) {
            ws.send(JSON.stringify({
                //datatype: "uavcan.equipment.indication.BeepCommand",
                //payload: {duration: 0.5, frequency: 440.0}
                audio: 1
            }));

            event.stopPropagation();
        } else if (event.target.classList.contains("esc-enumerate")) {
            /*
            Send an esc_index enumeration request to all active nodes;
            infinite timeout if enumeration is currently active, or zero
            timeout otherwise.
            */
            var nodes, i, nodeId;

            nodes = document.querySelectorAll(".device");
            highestEscIndex = lastEnumeratedNodeId = -1;
            enumerationActive = !enumerationActive;

            for (i = 0; i < nodes.length; i++) {
                if (nodes[i].id && nodes[i].id.indexOf("device-") === 0) {
                    nodeId = parseInt(nodes[i].id.split("-")[1], 10);

                    ws.send(JSON.stringify({
                        datatype: "uavcan.protocol.enumeration.Begin",
                        payload: {
                            parameter_name: "esc_index",
                            timeout_sec: enumerationActive ? 65535 : 0
                        },
                        node_id: nodeId
                    }));
                }
            }

            if (enumerationActive) {
                event.target.value = "Stop ESC enumeration";
            } else {
                event.target.value = "Start ESC enumeration";
            }
        } /* else if (nodeUi = selectAncestor(event.target, ".device-header")) {
            nodeUi.classList.toggle("collapsed");
            nodeUi.parentElement.querySelector(".device-detail")
                  .classList.toggle("hidden");
        } */
    });
}

connect();
setupEventListeners();


for (var i = 0; i < 16; i++) {
    escRawSetpointFunctions.push(null);
}
setInterval(updateSetpoint, 20);


function makeConstantFunction(tStart, value) {
    return function(t) { return value; }
}


function makeStepFunction(tStart, tLow, tRise, tHigh, tFall, minSetpoint, maxSetpoint) {
    return function(t) {
        var tTot = tLow + tRise + tHigh + tFall,
            tDelta = (t - tStart) % tTot,
            interp;

        if (tDelta <= tLow) {
            return minSetpoint;
        } else if (tDelta <= tLow + tRise) {
            interp = Math.min(1.0, (tDelta - tLow) / tRise);
            return minSetpoint + interp * (maxSetpoint - minSetpoint);
        } else if (tDelta <= tLow + tRise + tHigh) {
            return maxSetpoint;
        } else {
            interp = Math.min(1.0, (tDelta - tLow - tRise - tHigh) / tFall);
            return maxSetpoint + interp * (minSetpoint - maxSetpoint);
        }
    };
}


function makeSineFunction(tStart, freq, minSetpoint, maxSetpoint) {
    return function(t) {
        var phase = 2.0 * Math.PI * (t - tStart) * freq;

        return minSetpoint + (maxSetpoint - minSetpoint) * 0.5 *
                             (1.0 + Math.sin(phase));
    };
}


function makeSweepFunction(tStart, tSweep, minFreq, maxFreq, minSetpoint, maxSetpoint) {
    return function(t) {
        var t = (t - tStart) % (2.0 * tSweep),
            delta = 1.0 - Math.abs(1.0 - t / tSweep), /* 0-1-0 over t = 0-tSweep-tSweep*2 */
            phase = 2.0 * Math.PI * t *
                    (minFreq + (maxFreq - minFreq) * delta / 2.0);

        return minSetpoint + (maxSetpoint - minSetpoint) * 0.5 *
                             (1.0 + Math.sin(phase));
    };
}


function makeEscSetpointFunction(tStart, escUi) {
    var commandType = escUi.querySelector("select[name=command]").value;

    if (commandType == "constant") {
        var setpoint = parseFloat(escUi.querySelector("input[name=constant_setpoint]").value) || 0.0;

        return makeConstantFunction(tStart, setpoint);
    } else if (commandType == "step") {
        var tLow = parseFloat(escUi.querySelector("input[name=step_low_time]").value || 0.05),
            tRise = parseFloat(escUi.querySelector("input[name=step_rise_time]").value || 0.05),
            tHigh = parseFloat(escUi.querySelector("input[name=step_high_time]").value || 0.05),
            tFall = parseFloat(escUi.querySelector("input[name=step_fall_time]").value || 0.05),
            minSetpoint = parseFloat(escUi.querySelector("input[name=step_setpoint_start]").value || 0.0),
            maxSetpoint = parseFloat(escUi.querySelector("input[name=step_setpoint_end]").value || 0.0);

        return makeStepFunction(tStart, tLow, tRise, tHigh, tFall, minSetpoint, maxSetpoint);
    } else if (commandType == "sine") {
        var minSetpoint = parseFloat(escUi.querySelector("input[name=sine_setpoint_start]").value || 0.0),
            maxSetpoint = parseFloat(escUi.querySelector("input[name=sine_setpoint_end]").value || 0.0),
            freq = parseFloat(escUi.querySelector("input[name=sine_frequency]").value || 1.0);

        return makeSineFunction(tStart, freq, minSetpoint, maxSetpoint);
    } else if (commandType == "sweep") {
        var minSetpoint = parseFloat(escUi.querySelector("input[name=sweep_setpoint_start]").value || 0.0),
            maxSetpoint = parseFloat(escUi.querySelector("input[name=sweep_setpoint_end]").value || 0.0),
            minFreq = parseFloat(escUi.querySelector("input[name=sweep_frequency_start]").value || 1.0),
            maxFreq = parseFloat(escUi.querySelector("input[name=sweep_frequency_end]").value || 1.0),
            tSweep = parseFloat(escUi.querySelector("input[name=sweep_rise_time]").value || 1.0);

        return makeSweepFunction(tStart, tSweep, minFreq, maxFreq, minSetpoint, maxSetpoint);
    } else {
        return null;
    }
}
