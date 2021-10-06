<template>
    <div style="background-color: white; height: 100%">
        <h1>블랙 박스</h1>
        <button @click="clickBtn">블랙박스 ON</button>
        <canvas id="streamingCanvas" width="320" height="240"></canvas>
        <button @click="clickRecordBtn">녹화 시작</button>
        <button @click="clickStopBtn">녹화 중지</button>
    </div>
</template>

<script>

export default {
};
</script>

<style>
</style>