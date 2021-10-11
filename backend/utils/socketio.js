const fs = require("fs")
const path = require("path")
const fd = fs.openSync(path + 'map.txt', 'w+')
var outF = fs.createWriteStream(null, { flags: 'w', fd });
const { Mutex } = require('async-mutex');
const mutex = new Mutex()

// db
const db = require("./db.js");
const Record = require("./models/Record.js");
const Lost = require("./models/Lost.js");
const Picture = require("./models/Picture.js");

const moment = require('moment')
require('moment-timezone')
moment.tz.setDefault("Asia/Seoul")

var info = {
  mapq: [],
  map: {
    queue: [],
    data: [],
    dSizeY: 500,
    dSizeX: 500
  },
  robot: {
    battery: 100,
    velocity: 0,
    pos: [0, 0],
    mode: 0
  },
  log: [{ timestamp: moment().format('YYYY년 MM월 DD일 HH:mm:ss'), content: "from nodejs_emergency", emergency: 1, pose : { x: 0, y: 0 }   },
  { timestamp: moment().format('YYYY년 MM월 DD일 HH:mm:ss'), content: "from nodejs", emergency: 0, pose : { x: 20, y: 20 } },
  // {timestamp: moment().format('YYYY년 MM월 DD일 HH:mm:ss'), content: "from nodejs3", emergency: 0, pose : { x: 25, y: 25 } }
    ],
environment: {
  weather: "Cloudy",
    temperature: "30"
}
}
//* map
for (let y = 0; y < info.map.dSizeY; y++) {
  info.map.data.push([])
  for (let x = 0; x < info.map.dSizeX; x++) {
    info.map.data[y].push(-1)
  }
}

module.exports.createSocket = function (http_server) {
  const io = require("socket.io")(http_server, {
    cors: {
      origin: ["http://j5a102.p.ssafy.io", "http://localhost:8080", "http://localhost:8079"],
      methods: ["GET", "POST"],
      transports: ["websocket", "polling"],
      credentials: true,
    },
    allowEIO3: true,
    allowEI04: true,
  });


  const roomName = 'team'
  io.on("connection", async (socket) => {
    //소켓 서버 생성
    //(issue) socket.to(roomName) 하면 ROS 요청을 수신 못함

    socket.join(roomName)
    console.log("Connect from Client: ");

    socket.on("SocketNode2Server", (data) => {
      const { robot, environment } = data

      info.robot = robot
      // console.log(info.robot)
      info.environment = environment

    })

    //로직 1. 맵 이벤트
    socket.on("Map2ServerInit", async (data) => {
      //로직 1-1. 서버에 연결되기 전에 센싱했던 맵 데이터를 수신
        info.map.data[data[0]] = []
        for(i = 1; i<=500; i++) {
          info.map.data[data[0]].push(data[i])
        }
    });

    socket.on("Map2Server", async (data) => {
      //로직 1-1. 라이다로 센싱한 값 중 변동이 있는 셀 데이터만 전송
      // for (let i = 0; i < data.length; i++) {
        // const [y, x, next_value] = data[i]
        // if (info.map.data[y][x] != next_value) {
          // info.map.data[y][x] = next_value
          // info.map.queue.push([y, x, next_value])
        // }
      // }
      socket.to(roomName).emit('Map2Web', data)
      // console.log(info.map.queue.length, info.map.queue[info.map.queue.length-1])
      // console.log(info.map.queue.length)
      // release()
    });
    socket.on("MapInit", async () => {
      console.log("MapInit")
      socket.to(roomName).emit("MapInit")
    })

    //로직 1. 로봇 이벤트
    socket.on("Robot2Web", async (data) => {
      // console.log("emit robot to Web", info.robot)
      socket.emit('Robot2Web', info.robot);
    })

    //로직 2. 로그 이벤트
    socket.on("History2Server", async (data) => {
      // console.log("get Log from ROS")
      const date = moment().format('YYYY년 MM월 DD일 HH:mm:ss')
      const { content } = data
      if (!content) return
      info.log.push({ timestamp: date, content })
    });

    socket.on("History2Web", async (data) => {
      // console.log("emit log to web")
      socket.emit('History2Web', info.log);
    })

    //로직 4. 환경 이벤트
    socket.on("Environment2Web", async (data) => {
      // console.log("emit Environment to web")
      socket.emit('Environment2Web', info.environment);
    })

    // 실시간 화면 요청
    socket.on('request_stream_from_vue', () => {
      socket.to(roomName).emit('request_stream_from_node');
      // console.log("요청")
    });

    // 실시간 화면 응답
    socket.on('stream_from_python', (message) => {
      socket.to(roomName).emit('stream_from_nodejs', message);
      // console.log("응답")
    });

    // 블랙박스 저장
    socket.on('uploadVideo', async (url) => {
      const date = moment().format('YYYY년 MM월 DD일 HH:mm:ss')
      try {
        const newRecord = await Record.create({
          fileUrl: url,
          createdAt: date
        });
      } catch (error) {
        console.log(error);
      };
    });

    // 분실물 사진 저장
    socket.on('uploadImage', async (url) => {
      const date = moment().format('YYYY년 MM월 DD일 HH:mm:ss')
      try {
        const newLost = await Lost.create({
          fileUrl: url,
          createdAt: date
        });
      } catch (error) {
        console.log(error);
      };
    });

    // 웹 맵 클릭 좌표 수신
    socket.on('Click2Server', async (data) => {
      console.log("Click2Server")
      socket.to(roomName).emit('Click2Ros', data);
    });
  });
};