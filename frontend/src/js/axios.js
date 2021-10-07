const axios = require("axios")

const request = axios.create({
    baseURL: "http://j5a102.p.ssafy.io:3000",
    timeout: 3000
})


module.exports = {
    getLost: async () => {
        const res = await request.get("/api/Lost")
        return res["data"]["data"]
    },
}