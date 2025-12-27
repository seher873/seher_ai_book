import React from 'react';
import Layout from '@theme/Layout';

export default function NetworkTest() {
  return (
    <Layout title="Network Test" description="Network connectivity test page">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Network Connectivity Test</h1>
            <p>This page confirms that your Docusaurus application is running correctly.</p>
            
            <h2>Server Status</h2>
            <p>
              <span style={{color: 'green', fontWeight: 'bold'}}>✓ Server is running</span>
            </p>
            
            <h2>Dashboard with Robot Image</h2>
            <p>
              Your dashboard page with the robot image is available at: 
              <a href="/dashboard"> /dashboard</a>
            </p>
            
            <h2>Troubleshooting</h2>
            <p>If you cannot access the site from your browser:</p>
            <ol>
              <li><strong>WSL Users:</strong> Access the site via your Windows browser at <code>http://localhost:3000</code></li>
              <li><strong>Cloud Environments:</strong> Use your platform's port forwarding feature to expose port 3000</li>
              <li><strong>Firewall:</strong> Check if your firewall is blocking connections to port 3000</li>
            </ol>
          
            <h2>Dashboard Preview</h2>
            <p>Here is your robot image from the dashboard:</p>
            <div style={{textAlign: 'center', padding: '20px'}}>
              <img
                src="/img/robot.png"
                alt="Robot Image on Dashboard"
                style={{width: '200px', height: 'auto', border: '1px solid #ddd', borderRadius: '8px'}}
              />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}