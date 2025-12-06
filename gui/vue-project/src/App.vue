<script setup>
import { BApp, BButton, BContainer, BCol, BRow } from 'bootstrap-vue-next'
import {reactive, ref} from 'vue'
import * as ROSLIB from 'roslib'

const bridge = reactive({connected: 'NO'});
const bridgeClass = ref('red_text')
const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

// When the Rosbridge server connects, fill the span with id "status" with "successful"
ros.on("connection", () => {
  bridge.connected = 'CONNECTED';
  bridgeClass.value = "green_text"
});

// When the Rosbridge server experiences an error, fill the "status" span with the returned error
ros.on("error", (error) => {
  bridge.connected = 'ERROR';
  bridgeClass.value = "red_text"
});

// When the Rosbridge server shuts down, fill the "status" span with "closed"
ros.on("close", () => {
    bridge.connected = 'DISCONNECTED';
    bridgeClass.value = "red_text"
});

</script>

<template>
  <BApp>
    <div>
      <header class="row">
        <div>
          <h1>Bale Scanner</h1>
        </div>
      </header>

      <main class="w-100 row mx-auto">
        <BContainer fluid="md" class="row h-100 content my-3">
          <BCol id="command_box">
            <BRow class="mb-3">
              <BButton>Learn warehouse</BButton>
            </BRow>
            <BRow class="mb-3">
              <BButton>Scan for a single <i>half-unit</i></BButton>
            </BRow>
            <BRow class="mb-3">
              <BButton>Scan bales in real time</BButton>
            </BRow>
          </BCol>

          <BCol id="view_box">
            <p>Hola</p>
          </BCol>

        </BContainer>

        <BContainer id="status_box" class="row">
          <p>Rosbridge status: <b :class="bridgeClass">{{bridge.connected}}</b></p>
        </BContainer>
      </main>

    </div>

  </BApp>


</template>

<style scoped>
.content {
  background-color: rgb(230, 230, 230);
}


main {
  min-width: 80vw;
}

header {
  line-height: 1.5;
}


.red_text{
  color: red;
}
.green_text{
  color: rgb(4, 173, 4);
}

</style>
