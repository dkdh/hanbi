pipeline {
	agent any
	stages {
		stage('vue-build-deploy') {   
            steps { 
				script {
					try {
                        mattermostSend (
                            color: "#ff7f00",
                            message: "${env.JOB_NAME} #${env.BUILD_NUMBER}: vue-build-deploy 단계 시작 (<${env.BUILD_URL}|Link to build>)"
                        )                   
						sh 'docker rm -f vue-nginx'
						sh 'docker build -t dockerize-vuejs-app ./frontend'
						sh 'docker run --name vue-nginx -d -p 8080:8080 dockerize-vuejs-app'
						wrewrewrw
                    } catch(e) {
                        currentBuild.result = "FAILURE"
                    } finally {
                        if(currentBuild.result == "FAILURE") {
                            mattermostSend (
                                color: "danger",
    	                        message: "${env.JOB_NAME} #${env.BUILD_NUMBER}: vue-build-deploy 단계 실패 (<${env.BUILD_URL}|Link to build>)"
                            )
                        } else {
                            mattermostSend (
                                color: "good",
    	                        message: "${env.JOB_NAME} #${env.BUILD_NUMBER}: vue-build-deploy 단계 성공 (<${env.BUILD_URL}|Link to build>)"
                            )
                        }                            
                    }           

				} 
			}
		}
		stage('node-build-deploy') {   
            steps { 
				script {
					try {
                        mattermostSend (
                            color: "#ff7f00",
                            message: "${env.JOB_NAME} #${env.BUILD_NUMBER}: node-build-deploy 단계 시작 (<${env.BUILD_URL}|Link to build>)"
                        )                   
						sh 'docker rm -f node'
						sh 'docker build -t dockerize-nodejs-app ./backend'
						sh 'docker run --name node -d -p 3000:3000 dockerize-nodejs-app' 
                    } catch(e) {
                        currentBuild.result = "FAILURE"
                    } finally {
                        if(currentBuild.result == "FAILURE") {
                            mattermostSend (
                                color: "danger",
    	                        message: "${env.JOB_NAME} #${env.BUILD_NUMBER}: node-build-deploy 단계 실패 (<${env.BUILD_URL}|Link to build>)"
                            )
                        } else {
                            mattermostSend (
                                color: "good",
    	                        message: "${env.JOB_NAME} #${env.BUILD_NUMBER}: node-build-deploy 단계 성공 (<${env.BUILD_URL}|Link to build>)"
                            )
                        }                            
                    }           
				} 
			}
		} 
	} 
}     