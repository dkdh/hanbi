export default {
    namespaced: true,
    state: {
        log: [
        ], 
        num : {
            "robot" : 1,
            "emergency" : 0,
            "event" : 0
        }
    },
    mutations: {
        setLog(state, data) {
            state.log = data;
            console.log(state.log);
        }
    },
    actions: {
        setLog({ state, }, data) {
            // vuex log 데이터를 갱신하는 함수
            // 추가된 log가 없으면 반환
            // console.log("setLog on", state.log.length, data.length)
            if(state.log.length == data.length) return;

            // 이벤트 종류별 개수 세기
            state.num["emergency"] = 0
            state.num["event"] = 0

            for(let i=0; i <  data.length;i++) {
                if(data[i].emergency == 1) state.num["emergency"] +=1
                else state.num["event"]+=1
            }

            //vuex 데이터 갱신
            console.log("setLog : commit ", data)
            // commit('setLog', data);
            state.log = data

            //Map log 랜더링
        },
    }
}