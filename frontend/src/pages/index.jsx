
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A comprehensive guide to building embodied AI systems">
      <main>
        <div style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          minHeight: '70vh',
          padding: '20px',
          textAlign: 'center',
          backgroundColor: '#f9f9f9'
        }}>
          <div style={{maxWidth: '800px', margin: '0 auto'}}>
            <h1 style={{color: '#da27e0', fontSize: '2.5rem', marginBottom: '20px'}}>
              Physical AI & Humanoid Robotics
            </h1>
            <p style={{fontSize: '1.2rem', color: '#333', marginBottom: '30px'}}>
              <em>A Comprehensive Guide to Building Embodied AI Systems</em>
            </p>
            
            <div style={{
              display: 'flex',
              justifyContent: 'center',
              marginBottom: '30px'
            }}>
              <img
                src="/img/robot.png"
                alt="Physical AI Robot"
                style={{width: '250px', height: 'auto', borderRadius: '10px'}}
              />
            </div>
            
            <p style={{fontSize: '1.1rem', marginBottom: '30px', lineHeight: '1.6',textDecorationColor: '#da27e0'}}>
              Welcome to the cutting-edge world of Physical AI and Humanoid Robotics! 
              This comprehensive textbook serves as your complete guide to developing 
              intelligent systems that interact with the real world through robotic platforms.
            </p>
            
            <div style={{display: 'flex', justifyContent: 'center', gap: '20px', flexWrap: 'wrap', marginTop: '30px'}}>
              <Link
                to="/docs/intro"
                className="button button--primary button--lg"
                style={{margin: '5px'}}
              >
                Read Textbook
              </Link>
              <Link
                to="/dashboard"
                className="button button--secondary button--lg"
                style={{margin: '5px'}}
              >
                View Dashboard
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}