export default {
    namespaced: true,
    state: {
        log: [
            { timestamp: String(new Date()), content: "vuex" , emergency : 0, pose :{x:0,y:50}},
            { timestamp: String(new Date()), content: "vuex2", emergency : 0, pose :{x:0,y:50} },
            { timestamp: String(new Date()), content: "vuex3", emergency : 1, pose :{x:0,y:50} }
        ]
    },
    mutations: {
        
    },
    actions: {
        setLog({ state}, data) {
            state, data
            // if(state.log !== data) dispatch(renderLog,data)
            state.log = data
        },
        renderLog({state, rootState},data) {
            state, data
            const {dSizeY, dSizeX,resol} = rootState.Map
            const {log} = state
            let logDOM = document.querySelectorAll('.event');
            for(let i=0; i <  logDOM.length;i++) {
                const {y,x} = log[i].pose
                logDOM[i].style.top = (y/resol + dSizeY/2 ) +"px"
                logDOM[i].style.left = (x/resol + dSizeX/2)+"px"
                logDOM[i].style.backgroundColor = rootState.colors.event
            }
        }
    }
}