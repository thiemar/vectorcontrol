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
    deviceHFICharts = {}, deviceVoltageTempCharts = {},
    deviceAnimationCallbacks = {}, deviceOutputVoltageCharts = {},
    setpointTimer = null, lastUpdate = null;

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
        if (!nodeUi) {
            content = document.getElementById("content");
            nodeUi = document.getElementById("device-template").cloneNode(true);
            nodeUi.id = "device-" + message.node_id;
            nodeUi.querySelector("span.node-id").textContent = message.node_id;
            content.appendChild(nodeUi);
            setupSpeedChart(nodeUi);
            setupCurrentChart(nodeUi);
            setupVoltageTempChart(nodeUi);
            setupHFIChart(nodeUi);
            setupOutputVoltageChart(nodeUi);
        }

        /* Process message data */
        if (message.datatype == "uavcan.protocol.NodeStatus") {
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
        } else if (message.datatype == "uavcan.protocol.GetNodeInfo") {
            nodeUi.querySelector("span.node-name").textContent =
                message.payload.name;
        } else if (message.datatype == "uavcan.protocol.param.GetSet") {
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

            updateCurrentChart(nodeUi, nodeData[message.node_id] || []);
            updateSpeedChart(nodeUi, nodeData[message.node_id] || []);
            updateOutputVoltageChart(nodeUi,
                                     nodeData[message.node_id] || []);
        } else if (message.datatype == "uavcan.equipment.esc.FOCStatus") {
            /*
            Measurement data -- add it to the measurement array, removing old
            data if the total length is more than 600 samples
            (30 seconds * 20 Hz). Update the charts once done.
            */
            if (!nodeData[message.node_id]) {
                nodeData[message.node_id] = [];
            }
            nodeData[message.node_id].push(message.payload);
            if (nodeData[message.node_id].length > 600) {
                nodeData[message.node_id] =
                    nodeData[message.node_id].slice(
                        nodeData[message.node_id].length - 600);
            }

            if (!deviceAnimationCallbacks[message.node_id]) {
                deviceAnimationCallbacks[message.node_id] =
                    requestAnimationFrame(function () {
                        updateCharts(nodeUi, nodeData[message.node_id]);
                    });
            }
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
    }, 1000.0);
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


function setupHFIChart(device) {
    var result = { chart: null, x: null, y: null, xAxis: null, yAxis: null },
        svg = device.querySelector("svg.hfi-chart"),
        container = device.querySelector("div.device-measurements"),
        width = container.clientWidth - 150,
        height = 200,
        margin = {top: 10, right: 50, left: 50, bottom: 10};

    result.x = d3.scale.linear()
        .range([0, width])
        .domain([0.0, 30.0]);

    result.y0 = d3.scale.linear()
        .range([height, 0.0])
        .domain([0.0, 0.5]);

    result.y1 = d3.scale.linear()
        .range([height, 0.0])
        .domain([0, 360.0]);

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
        .text("HFI signal (A)");

    result.chart.append("g")
        .attr("class", "y1 axis angle")
        .attr("transform", "translate(" + width + ", 0)")
        .call(result.yAxis1)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", 40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Angle (°)");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "hfi-angle")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "hfi-d")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "hfi-q")
        .attr('clip-path', 'url(#clip)');

    deviceHFICharts[parseInt(device.id.split("-")[1], 10)] = result;
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
        .attr("class", "speed-setpoint")
        .attr('clip-path', 'url(#clip)');

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

    result.y0 = d3.scale.linear()
        .range([height, 0.0])
        .domain([-30.0, 30.0]);

    result.y1 = d3.scale.linear()
        .range([height, 0.0])
        .domain([0, 1.0]);

    result.xAxis = d3.svg.axis()
        .scale(result.x)
        .orient("bottom")
        .ticks(30)
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
        .call(result.xAxis)
    .append("text")
        .attr("x", width / 2)
        .attr("y", 30)
        .style("text-anchor", "middle")
        .text("Time (s)");

    result.chart.append("g")
        .attr("class", "y0 axis")
        .call(result.yAxis0)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", -40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Output voltage (V)");

    result.chart.append("g")
        .attr("class", "y1 axis consistency")
        .attr("transform", "translate(" + width + ", 0)")
        .call(result.yAxis1)
    .append("text")
        .attr("x", -height / 2)
        .attr("y", 40)
        .style("text-anchor", "middle")
        .attr("transform", "rotate(-90)")
        .text("Estimator consistency");

    result.chart.append("clipPath")
        .attr("id", "clip")
    .append("rect")
        .attr("width", width)
        .attr("height", height);

    result.chart.append("path")
        .attr("class", "consistency-unit")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "voltage-vd")
        .attr('clip-path', 'url(#clip)');

    result.chart.append("path")
        .attr("class", "voltage-vq")
        .attr('clip-path', 'url(#clip)');

    deviceOutputVoltageCharts[parseInt(device.id.split("-")[1], 10)] = result;
}

function updateCharts(device, data) {
    var deviceId;
    deviceId = parseInt(device.id.split("-")[1], 10);
    deviceAnimationCallbacks[deviceId] = undefined;

    updateCurrentChart(deviceId, device, data);
    updateSpeedChart(deviceId, device, data);
    updateVoltageTempChart(deviceId, device, data);
    updateHFIChart(deviceId, device, data);
    updateOutputVoltageChart(deviceId, device, data);
}

function updateVoltageTempChart(deviceId, device, data) {
    var seriesData, chart;

    chart = deviceVoltageTempCharts[deviceId];

    /* Vbus line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y0(d.vbus); });

    chart.chart.select(".vbus")
        .datum(data)
        .attr("d", seriesData);

    /* Temperature line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y1(d.temperature - 273.15); });

    chart.chart.select(".temperature")
        .datum(data)
        .attr("d", seriesData);
}

function updateCurrentChart(deviceId, device, data) {
    var current, chart, maxCurrent;

    chart = deviceCurrentCharts[deviceId];
    maxCurrent = parseFloat(device.querySelector("input[name=motor_current_limit]").value) || 40.0;

    chart.y.domain([-maxCurrent, maxCurrent]);
    chart.yAxis.scale(chart.y);
    chart.chart.select(".y.axis").call(chart.yAxis);

    /* Setpoint line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.i_setpoint); });

    chart.chart.select(".current-setpoint")
        .datum(data)
        .attr("d", current);

    /* Id line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.i_dq[0]); });

    chart.chart.select(".current-id")
        .datum(data)
        .attr("d", current);

    /* Iq line */
    current = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.i_dq[1]); });

    chart.chart.select(".current-iq")
        .datum(data)
        .attr("d", current);
}


function updateSpeedChart(deviceId, device, data) {
    var speed, chart, maxSpeed;

    chart = deviceSpeedCharts[deviceId];
    maxSpeed = parseFloat(device.querySelector("input[name=motor_rpm_max]").value) || 10000.0;

    chart.y.domain([-maxSpeed * 1.1, maxSpeed * 1.1]);
    chart.yAxis.scale(chart.y);
    chart.chart.select(".y.axis").call(chart.yAxis);

    /* Setpoint line */
    speed = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.rpm_setpoint); });

    chart.chart.select(".speed-setpoint")
        .datum(data)
        .attr("d", speed);

    /* Id line */
    speed = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y(d.rpm); });

    chart.chart.select(".speed-actual")
        .datum(data)
        .attr("d", speed);
}


function updateHFIChart(deviceId, device, data) {
    var seriesData, chart;

    chart = deviceHFICharts[deviceId];

    /* D current line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y0(Math.sqrt(d.hfi_dq[0])); });

    chart.chart.select(".hfi-d")
        .datum(data)
        .attr("d", seriesData);

    /* Q current line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y0(Math.sqrt(d.hfi_dq[1])); });

    chart.chart.select(".hfi-q")
        .datum(data)
        .attr("d", seriesData);

    /* Angle line */
    seriesData = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y1(d.angle / 255.0 * 360.0); });

    chart.chart.select(".hfi-angle")
        .datum(data)
        .attr("d", seriesData);
}


function updateOutputVoltageChart(deviceId, device, data) {
    var voltage, chart, maxVoltage, consistency;

    chart = deviceOutputVoltageCharts[deviceId];
    maxVoltage = parseFloat(device.querySelector("input[name=motor_voltage_limit]").value) || 27.0;

    chart.y0.domain([-maxVoltage, maxVoltage]);
    chart.yAxis0.scale(chart.y0);
    chart.chart.select(".y0.axis").call(chart.yAxis0);

    /* Consistency line */
    consistency = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y1(d.consistency / 255.0); });

    chart.chart.select(".consistency-unit")
        .datum(data)
        .attr("d", consistency);

    /* Vd line */
    voltage = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y0(d.v_dq[0]); });

    chart.chart.select(".voltage-vd")
        .datum(data)
        .attr("d", voltage);

    /* Vq line */
    voltage = d3.svg.line()
        .x(function(d, i) { return chart.x(i / 20.0); })
        .y(function(d) { return chart.y0(d.v_dq[1]); });

    chart.chart.select(".voltage-vq")
        .datum(data)
        .attr("d", voltage);
}


function updateSetpoint() {
    var schedule, setpoints = [], escIndex;

    switch (document.querySelector("select[name=command]").value) {
        case "constant":
            schedule = [parseFloat(document.querySelector("input[name=constant_setpoint]").value) || 0.0]
            break;
        case "step":
            tLow = parseFloat(document.querySelector("input[name=step_low_time]").value || 0.01);
            tRise = parseFloat(document.querySelector("input[name=step_rise_time]").value || 0.01);
            tHigh = parseFloat(document.querySelector("input[name=step_high_time]").value || 0.01);
            tFall = parseFloat(document.querySelector("input[name=step_fall_time]").value || 0.01);
            minSetpoint = parseFloat(document.querySelector("input[name=step_setpoint_start]").value || 0.0);
            maxSetpoint = parseFloat(document.querySelector("input[name=step_setpoint_end]").value || 0.0);
            tTotal = tLow + tRise + tHigh + tFall;

            schedule = [];
            for (i = 0; i < tTotal; i += 0.01) {
                if (i < tLow) {
                    schedule.push(minSetpoint);
                } else if (i >= tLow && i < tLow + tRise) {
                    schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                                (i - tLow) / tRise);
                } else if (i >= tLow + tRise && i < tLow + tRise + tHigh) {
                    schedule.push(maxSetpoint);
                } else if (i >= tLow + tRise + tHigh) {
                    schedule.push(maxSetpoint + (minSetpoint - maxSetpoint) *
                                                (i - tLow - tRise - tHigh) /
                                                tFall);
                }
            }
            break;
        case "sine":
            minSetpoint = parseFloat(document.querySelector("input[name=sine_setpoint_start]").value || 0.0);
            maxSetpoint = parseFloat(document.querySelector("input[name=sine_setpoint_end]").value || 0.0);
            minFreq = parseFloat(document.querySelector("input[name=sine_frequency]").value || 1.0);
            tTotal = 1.0 / minFreq;

            schedule = [];
            for (i = 0, p = 0, d = 2.0 * Math.PI * minFreq / 100.0;
                    i < tTotal; i += 0.01) {
                schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                            (0.5 * (1.0 + Math.sin(p))));
                p += d;
            }
            break;
        case "sweep":
            minSetpoint = parseFloat(document.querySelector("input[name=sweep_setpoint_start]").value || 0.0);
            maxSetpoint = parseFloat(document.querySelector("input[name=sweep_setpoint_end]").value || 0.0);
            minFreq = parseFloat(document.querySelector("input[name=sweep_frequency_start]").value || 1.0);
            maxFreq = parseFloat(document.querySelector("input[name=sweep_frequency_end]").value || 1.0);
            tTotal = parseFloat(document.querySelector("input[name=sweep_rise_time]").value || 1.0);

            schedule = [];
            for (i = 0, p = 0, f = minFreq, d = 2.0 * Math.PI * f / 100.0;
                    i < tTotal * 0.5; i += 0.01) {
                schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                            (0.5 * (1.0 + Math.sin(p))));
                p += d;
                f += (maxFreq - minFreq) / (100.0 * tTotal * 0.5);
                d = 2.0 * Math.PI * f / 100.0;
            }
            for (f = maxFreq, d = 2.0 * Math.PI * f / 100.0;
                    i < tTotal; i += 0.01) {
                schedule.push(minSetpoint + (maxSetpoint - minSetpoint) *
                                            (0.5 * (1.0 + Math.sin(p))));
                p += d;
                f += (minFreq - maxFreq) / (100.0 * tTotal * 0.5);
                d = 2.0 * Math.PI * f / 100.0;
            }

            break;
    }

    escIndex = document.querySelector("select[name=esc_index]").selectedIndex;

    for (var i = 0; i < 16; i++) {
        setpoints.push(0.0);
    }
    setpoints[escIndex] = schedule[0] || 0.0;

    if (document.querySelector("select[name='mode']").selectedIndex === 0) {
        /* Rescale setpoints to use the full duty cycle range */
        for (var i = 0; i < 16; i++) {
            setpoints[i] = parseInt(setpoints[i] * (8191.0 / 100.0), 10);
        }

        ws.send(JSON.stringify({
            datatype: "uavcan.equipment.esc.RawCommand",
            payload: {
                cmd: setpoints
            }
        }));
    } else {
        ws.send(JSON.stringify({
            datatype: "uavcan.equipment.esc.RPMCommand",
            payload: {
                rpm: setpoints
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
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement.parentElement;
            document.querySelectorAll("fieldset.command-params label")
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
        var nodeUi;

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
        } else if (event.target.classList.contains("esc-start")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement;
            nodeUi.querySelectorAll("input, select").forEach(function(elem) {
                if (elem.name == "stop") {
                    elem.disabled = false;
                } else {
                    elem.disabled = true;
                }
            });

            setpointTimer = setInterval(updateSetpoint, 100);
        } else if (event.target.classList.contains("esc-stop")) {
            nodeUi = event.target.parentElement.parentElement.parentElement
                                 .parentElement;
            nodeUi.querySelectorAll("input, select").forEach(function(elem) {
                if (elem.name == "stop") {
                    elem.disabled = true;
                } else {
                    elem.disabled = false;
                }
            });

            clearInterval(setpointTimer);
            setpointTimer = null;
        } /* else if (nodeUi = selectAncestor(event.target, ".device-header")) {
            nodeUi.classList.toggle("collapsed");
            nodeUi.parentElement.querySelector(".device-detail")
                  .classList.toggle("hidden");
        } */
    });
}

connect();
setupEventListeners();

