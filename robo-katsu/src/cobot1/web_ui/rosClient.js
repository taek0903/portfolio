(function attachRosClient(global) {
  const ROSLIB_SCRIPT_ID = 'roslib-script';
  const ROSLIB_SRC = 'https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js';

  let roslibPromise = null;

  const loadRoslib = () => {
    if (global.ROSLIB) {
      return Promise.resolve(global.ROSLIB);
    }

    if (roslibPromise) {
      return roslibPromise;
    }

    roslibPromise = new Promise((resolve, reject) => {
      let script = document.getElementById(ROSLIB_SCRIPT_ID);

      const handleLoad = () => {
        if (global.ROSLIB) {
          resolve(global.ROSLIB);
          return;
        }

        roslibPromise = null;
        reject(new Error('ROSLIB failed to initialize.'));
      };

      const handleError = () => {
        roslibPromise = null;
        reject(new Error('Failed to load ROSLIB.'));
      };

      if (!script) {
        script = document.createElement('script');
        script.id = ROSLIB_SCRIPT_ID;
        script.src = ROSLIB_SRC;
        script.async = true;
        script.crossOrigin = 'anonymous';
        (document.body || document.head).appendChild(script);
      }

      script.addEventListener('load', handleLoad, { once: true });
      script.addEventListener('error', handleError, { once: true });
    });

    return roslibPromise;
  };

  class RosClient {
    constructor(url) {
      this.url = url;
      this.ros = null;
    }

    async connect(handlers = {}) {
      await loadRoslib();

      if (this.ros) {
        this.disconnect();
      }

      this.ros = new global.ROSLIB.Ros({ url: this.url });

      if (handlers.onConnection) {
        this.ros.on('connection', handlers.onConnection);
      }

      if (handlers.onError) {
        this.ros.on('error', handlers.onError);
      }

      if (handlers.onClose) {
        this.ros.on('close', handlers.onClose);
      }

      return this.ros;
    }

    publish(topicName, messageType, payload) {
      if (!this.ros || !global.ROSLIB) {
        throw new Error('ROS connection is not ready.');
      }

      const topic = new global.ROSLIB.Topic({
        ros: this.ros,
        name: topicName,
        messageType,
      });

      topic.publish(new global.ROSLIB.Message(payload));
    }

    callService(serviceName, serviceType, requestPayload = {}) {
      if (!this.ros || !global.ROSLIB) {
        throw new Error('ROS connection is not ready.');
      }

      const service = new global.ROSLIB.Service({
        ros: this.ros,
        name: serviceName,
        serviceType,
      });

      const request = new global.ROSLIB.ServiceRequest(requestPayload);

      return new Promise((resolve, reject) => {
        try {
          service.callService(
            request,
            (response) => resolve(response),
            (error) => {
              if (error instanceof Error) {
                reject(error);
                return;
              }

              reject(new Error(typeof error === 'string' ? error : 'Service call failed.'));
            },
          );
        } catch (error) {
          reject(error);
        }
      });
    }

    subscribe(topicName, messageType, callback) {
      if (!this.ros || !global.ROSLIB) {
        throw new Error('ROS connection is not ready.');
      }

      const topic = new global.ROSLIB.Topic({
        ros: this.ros,
        name: topicName,
        messageType,
      });

      topic.subscribe(callback);

      return () => topic.unsubscribe(callback);
    }

    disconnect() {
      if (this.ros) {
        this.ros.close();
        this.ros = null;
      }
    }
  }

  global.createRosClient = (url) => new RosClient(url);
})(window);
