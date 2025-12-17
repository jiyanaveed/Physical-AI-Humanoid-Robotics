import axios, { AxiosInstance, AxiosResponse } from 'axios';

// Get the RAG API URL - using a default value since Docusaurus handles environment differently
const ragApiUrl = 'http://localhost:8000';
const timeout = 10000; // 10 second timeout

// Create the API client instance
const ragApiClient: AxiosInstance = axios.create({
  baseURL: ragApiUrl,
  timeout: timeout,
  headers: {
    'Content-Type': 'application/json',
  },
});


export default ragApiClient;