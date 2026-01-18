
import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';
import { useAuth } from '../components/Auth/AuthProvider'; // Using consistent auth system

export default function Dashboard() {
  const {siteConfig} = useDocusaurusContext();
  const { user, loading } = useAuth(); // Using the custom auth context

  if (loading) {
    return (
      <Layout
        title={`Dashboard - ${siteConfig.title}`}
        description="Physical AI Textbook Dashboard">
        <main className="container margin-vert--lg">
          <div style={{
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            minHeight: '70vh',
            fontSize: '18px'
          }}>
            Loading...
          </div>
        </main>
      </Layout>
    );
  }

  if (!user) {
    // Redirect to login if not authenticated
    if (typeof window !== 'undefined') {
      window.location.href = '/auth/login';
    }
    return null;
  }

  return (
    <Layout
      title={`Dashboard - ${siteConfig.title}`}
      description="Physical AI Textbook Dashboard">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <div style={{textAlign: 'center', padding: '40px 20px', backgroundColor: '#f9f9f9', borderRadius: '8px', marginBottom: '30px'}}>
              <h1 style={{color: '#da27e0', fontWeight: 'bold', fontSize: '2.5rem'}}>Welcome, {user.name || user.email}!</h1>
              <p style={{fontSize: '1.2rem', color: '#333'}}><em>Your central hub for learning and tracking progress</em></p>
            </div>

            <div style={{display: 'flex', justifyContent: 'center', alignItems: 'center', padding: '20px', marginBottom: '30px'}}>
              <img
                src="/img/roobot.png"
                alt="Physical AI Robot"
                style={{width: '300px', height: 'auto', borderRadius: '10px', boxShadow: '0 4px 8px rgba(0,0,0,0.2)'}}
              />
            </div>

            <div style={{display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(300px, 1fr))', gap: '20px', marginTop: '30px'}}>
              <div className={clsx('padding--md', 'card')} style={{backgroundColor: '#fff', border: '1px solid #eee', borderRadius: '8px'}}>
                <h3 style={{color: '#da27e0'}}>Learning Progress</h3>
                <p>Track your progress through the textbook chapters</p>
                <div className="progress-bar" style={{width: '100%', backgroundColor: '#e0e0e0', borderRadius: '4px', height: '20px', marginTop: '10px'}}>
                  <div style={{width: '25%', backgroundColor: '#da27e0', height: '100%', borderRadius: '4px'}}></div>
                </div>
                <p style={{marginTop: '10px'}}>25% Complete</p>
              </div>

              <div className={clsx('padding--md', 'card')} style={{backgroundColor: '#fff', border: '1px solid #eee', borderRadius: '8px'}}>
                <h3 style={{color: '#da27e0'}}>Recent Activity</h3>
                <ul style={{textAlign: 'left', paddingLeft: '20px'}}>
                  <li>Completed Chapter 1: Introduction to Physical AI</li>
                  <li>Started Chapter 2: Robot Perception Systems</li>
                  <li>Attempted Lab Exercise: Basic ROS2 Operations</li>
                </ul>
              </div>

              <div className={clsx('padding--md', 'card')} style={{backgroundColor: '#fff', border: '1px solid #eee', borderRadius: '8px'}}>
                <h3 style={{color: '#da27e0'}}>Quick Links</h3>
                <ul style={{textAlign: 'left', paddingLeft: '20px'}}>
                  <li><Link to="/docs/intro">Textbook Contents</Link></li>
                  <li><Link to="/docs/intro">Introduction</Link></li>
                  <li><Link to="/dashboard">Dashboard</Link></li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}