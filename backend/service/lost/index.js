// var fs = require('fs')
// const os = require('os')
exports.get_lost = (req, res) => {
    return new Promise(async (resolve, reject) => {
        try {
            //분실물 목록을 반환하는 서비스

            //로직 1-1. DB에서 분실물 가져오기

            //로직 1-2. 임시 데이터 
            const lost = [{ name: "열쇠", date: new Date(), img: 0 }, { name: "지갑", date: new Date(), img: 0 }, { name: "소소기찜덮밥", date: new Date(), img: 0 }]

            return resolve(lost)
        } catch (error) {
            return reject(error);
        }
    })
}