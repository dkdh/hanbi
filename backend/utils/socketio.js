const fs = require("fs")
const params = require("../config")
path = "H:\\Project\\Ssafy_2\\PJT2\\sbj3\\S05P21A102\\backend\\assets\\"
const fd = fs.openSync(path + 'map.txt', 'w+')
var outF = fs.createWriteStream(null, { flags: 'w', fd });

mapData = []

module.exports.createSocket = function (http_server) {
  const io = require("socket.io")(http_server, {
    cors: {
      origin: params["origin"],
      methods: ["GET", "POST"],
      transports: ["websocket", "polling"],
      credentials: true,
    },
    // allowEIO3: true,
    allowEI04: true,
  });


  const roomName = 'team'
  io.on("connection", async (socket) => {
    socket.join(roomName)
    console.log("Connect from Client: ");

    socket.on("setMap2Server", async (data) => {
      mapData = data
      socket.to(roomName).emit('setMap2Web', mapData);
    });

  });
};
